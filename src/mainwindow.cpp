#include "mainwindow.h"
#include "gaussian_blur.h"
#include "imageprocessor.h"
#include "edgesharpener.h"
#include <QApplication>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QFileDialog>
#include <QMessageBox>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QRadioButton>
#include <QButtonGroup>
#include <QPainter>
#include <QFile>
#include <QFileInfo>
#include <QImageReader>
#include <QMenuBar>
#include <QMenu>
#include <QAction>
#include <QDialog>
#include <QDialogButtonBox>
#include <QSlider>
#include <QLabel>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QSignalBlocker>
#include <QGroupBox>
#include <QScrollArea>
#include <QProgressBar>
#include <QStatusBar>
#include <QSplitter>
#include <QDir>
#include <QDirIterator>
#include <QCoreApplication>
#include <QTimer>
#include <QDateTime>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonArray>
#include <QSaveFile>
#include <QTemporaryDir>
#include <QProcess>
#include <QFutureWatcher>
#include <QtConcurrent/QtConcurrent>
#include <cmath>
#include <QDebug>

// ============================================================
//  静态工具函数
// ============================================================
namespace {

static int clampOddKernel(int value, int minValue, int maxValue) {
    int v = qBound(minValue, value, maxValue);
    if ((v % 2) == 0) v = (v >= maxValue) ? v - 1 : v + 1;
    return qBound(minValue, v, maxValue);
}

static QImage posterizeImage(const QImage& srcImage, int levels) {
    if (srcImage.isNull()) return srcImage;
    const int safeLevels = qBound(2, levels, 64);
    const int stepCount = safeLevels - 1;
    QImage src = srcImage.convertToFormat(QImage::Format_RGB32);
    QImage dst(src.size(), QImage::Format_RGB32);
    auto quantize = [stepCount](int v) {
        return qBound(0, (v * stepCount + 127) / 255 * 255 / stepCount, 255);
    };
    for (int y = 0; y < src.height(); ++y) {
        const QRgb* in = reinterpret_cast<const QRgb*>(src.constScanLine(y));
        QRgb* out = reinterpret_cast<QRgb*>(dst.scanLine(y));
        for (int x = 0; x < src.width(); ++x)
            out[x] = qRgb(quantize(qRed(in[x])), quantize(qGreen(in[x])), quantize(qBlue(in[x])));
    }
    return dst;
}

static QImage preprocessPhotoForLayers(const QImage& srcImage, bool denoiseEnabled, bool posterizeEnabled, int kernelSize, int posterizeLevels) {
    if (srcImage.isNull()) return srcImage;
    QImage result = srcImage;
    // Step 1: denoise (gaussian blur) — always from original
    if (denoiseEnabled) {
        const int kernel = clampOddKernel(kernelSize, 3, 11);
        const double sigma = qMax(0.8, kernel / 3.0);
        result = applyGaussianBlur(result, kernel, sigma).convertToFormat(QImage::Format_RGB32);
    }
    // Step 2: posterize (color quantization)
    if (posterizeEnabled) {
        result = posterizeImage(result, qBound(2, posterizeLevels, 64));
    }
    return result;
}

// ---------- BFS 连通域清理 ----------
// Safe iterative BFS. Stack is bounded by maxTrack to prevent OOM on large images.
// Uses a pre-built gray lookup to avoid repeated scanLine calls and COW detach races.
static void cleanLayerComponents(QImage& image, bool targetWhite, int minArea) {
    if (image.isNull() || minArea <= 1) return;

    // Work on a stable RGB32 copy — avoids Qt COW detach issues during BFS
    QImage work = image.convertToFormat(QImage::Format_RGB32);
    const int w = work.width();
    const int h = work.height();

    // Pre-build flat gray array to avoid scanLine during BFS inner loop
    QVector<uchar> gray(w * h);
    for (int y = 0; y < h; ++y) {
        const QRgb* line = reinterpret_cast<const QRgb*>(work.constScanLine(y));
        uchar* gline = &gray[y * w];
        for (int x = 0; x < w; ++x)
            gline[x] = static_cast<uchar>(qGray(line[x]) >= 128 ? 255 : 0);
    }

    // BFS safety cap: stop tracking when component exceeds this size (treat as large = keep)
    const int maxTrack = qMin(minArea * 8, 262144);

    QVector<uchar> seen(w * h, 0);
    QVector<int> stack;
    QVector<int> component;
    stack.reserve(qMin(maxTrack, 16384));
    component.reserve(qMin(maxTrack, 16384));

    const uchar target = targetWhite ? 255 : 0;

    for (int y0 = 0; y0 < h; ++y0) {
        for (int x0 = 0; x0 < w; ++x0) {
            const int startIdx = y0 * w + x0;
            if (seen[startIdx] || gray[startIdx] != target) continue;

            seen[startIdx] = 1;
            stack.clear();
            component.clear();
            stack.append(startIdx);
            bool oversized = false;

            while (!stack.isEmpty()) {
                // Enforce stack size cap — if stack itself gets too big, mark oversized
                if (stack.size() + component.size() >= maxTrack) {
                    // Mark everything in stack as seen and bail
                    for (int idx : stack) seen[idx] = 1;
                    stack.clear();
                    oversized = true;
                    break;
                }

                const int idx = stack.takeLast();
                component.append(idx);
                const int cx = idx % w;
                const int cy = idx / w;

                // 4-connected neighbors
                if (cx > 0)     { const int n = idx-1;   if (!seen[n] && gray[n]==target) { seen[n]=1; stack.append(n); } }
                if (cx < w-1)   { const int n = idx+1;   if (!seen[n] && gray[n]==target) { seen[n]=1; stack.append(n); } }
                if (cy > 0)     { const int n = idx-w;   if (!seen[n] && gray[n]==target) { seen[n]=1; stack.append(n); } }
                if (cy < h-1)   { const int n = idx+w;   if (!seen[n] && gray[n]==target) { seen[n]=1; stack.append(n); } }
            }

            // Small component (not oversized, not touching border for white) → erase
            if (oversized || component.size() >= minArea) continue;

            // Check border touch for white components (don't erase border-connected white)
            if (targetWhite) {
                bool border = false;
                for (int idx : component) {
                    const int cx = idx % w, cy = idx / w;
                    if (cx == 0 || cy == 0 || cx == w-1 || cy == h-1) { border = true; break; }
                }
                if (border) continue;
            }

            // Erase: update both gray array and work image
            const uchar fill = targetWhite ? 0 : 255;
            const QRgb fillRgb = targetWhite ? 0xFF000000 : 0xFFFFFFFF;
            for (int idx : component) {
                gray[idx] = fill;
                const int cx = idx % w, cy = idx / w;
                reinterpret_cast<QRgb*>(work.scanLine(cy))[cx] = fillRgb;
            }
        }
    }

    image = work;
}

static void cleanBinaryLayer(QImage& image, bool enabled, int minArea) {
    if (!enabled || image.isNull()) return;
    const int safeArea = qBound(2, minArea, 5000);
    cleanLayerComponents(image, true,  safeArea);
    cleanLayerComponents(image, false, safeArea);
}

// ---------- 预览坐标计算 ----------
static QRectF calcPreviewRect(const QSize& labelSize, const QSize& imageSize, double zoom, const QPointF& pan) {
    if (!labelSize.isValid() || !imageSize.isValid() || imageSize.isEmpty()) return QRectF();
    const double sx = static_cast<double>(labelSize.width())  / imageSize.width();
    const double sy = static_cast<double>(labelSize.height()) / imageSize.height();
    const double fit = qMin(sx, sy);
    const double drawW = imageSize.width()  * fit * zoom;
    const double drawH = imageSize.height() * fit * zoom;
    const double x = (labelSize.width()  - drawW) * 0.5 + pan.x();
    const double y = (labelSize.height() - drawH) * 0.5 + pan.y();
    return QRectF(x, y, drawW, drawH);
}

static qint64 getMaxImportPixels() { return 16000000LL; }

static QSize scaleDownToPixelLimit(const QSize& src, qint64 maxPixels) {
    if (!src.isValid() || src.isEmpty() || maxPixels <= 0) return QSize();
    const qint64 srcPixels = static_cast<qint64>(src.width()) * static_cast<qint64>(src.height());
    if (srcPixels < maxPixels) return src;
    const double scale = std::sqrt(static_cast<double>(maxPixels - 1) / static_cast<double>(srcPixels));
    int w = qMax(1, static_cast<int>(std::floor(src.width()  * scale)));
    int h = qMax(1, static_cast<int>(std::floor(src.height() * scale)));
    while (static_cast<qint64>(w) * static_cast<qint64>(h) >= maxPixels && (w > 1 || h > 1)) {
        if (w >= h && w > 1) --w; else if (h > 1) --h; else break;
    }
    return QSize(w, h);
}

static QString detectImageExtensionFromContent(const QString& filePath) {
    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) return QString();
    QByteArray hdr = f.read(16);
    f.close();
    if (hdr.size() >= 8 && hdr.startsWith("\x89PNG\r\n\x1A\n")) return QStringLiteral("png");
    if (hdr.size() >= 3 && (uchar)hdr[0]==0xFF && (uchar)hdr[1]==0xD8 && (uchar)hdr[2]==0xFF) return QStringLiteral("jpg");
    if (hdr.startsWith("GIF87a") || hdr.startsWith("GIF89a")) return QStringLiteral("gif");
    if (hdr.size() >= 2 && hdr[0]=='B' && hdr[1]=='M') return QStringLiteral("bmp");
    if (hdr.size() >= 12 && hdr.startsWith("RIFF") && hdr.mid(8,4)=="WEBP") return QStringLiteral("webp");
    return QString();
}

static bool isSupportedImageExtension(const QString& extLower) {
    return extLower=="png"||extLower=="jpg"||extLower=="jpeg"||extLower=="bmp"||extLower=="gif"||extLower=="webp";
}

static QString psSingleQuoted(const QString& value) {
    QString escaped = value; escaped.replace("'","''");
    return QString("'%1'").arg(escaped);
}

static qint64 fileStampMs(const QFileInfo& fi) {
    return fi.exists() ? fi.lastModified().toMSecsSinceEpoch() : -1;
}

} // namespace

// ============================================================
//  后台静态处理函数（无 Qt 对象，线程安全）
// ============================================================
ProcessResult MainWindow::runProcessing(ProcessParams p) {
    ProcessResult res;
    if (p.srcImage.isNull()) return res;

    // 1. 照片预处理（每次从原图重新算，不修改原图）
    QImage working = preprocessPhotoForLayers(
        p.srcImage, p.denoiseEnabled, p.posterizeEnabled,
        p.photoPreprocessKernelSize, p.photoPosterizeLevels);

    // 2. 边缘掩码（可选）
    QImage edgeMask;
    if (p.edgeEnabled) {
        EdgeSharpener sharpener;
        edgeMask = sharpener.buildEdgeMaskForImage(
            working, p.edgeMode,
            p.edgeThreshMin, p.edgeThreshMax,
            p.edgePrefilterEnabled, p.edgePrefilterKernelSize, p.edgePrefilterSigma);
    }

    // 3. 像素分类
    ImageProcessor proc;
    proc.processImage(
        working,
        p.goldThresh, p.silkThresh, p.transThresh, p.copperDepth,
        p.ledRadVal,
        p.maskColorName, p.finishType, p.isWhiteMask,
        p.enableBareSubstrate, p.bareSubstrateUseGrayBinding,
        p.bareSubstrateGrayMinPct, p.bareSubstrateGrayMaxPct,
        p.bareSubstrateColorSimilarityPct,
        res.copper, res.mask, res.silk, res.bottom, res.composite,
        p.ledStrips, false);

    // 4. 叠加边缘掩码
    if (!edgeMask.isNull()) {
        const int w = qMin(edgeMask.width(), res.copper.width());
        const int h = qMin(edgeMask.height(), res.copper.height());
        const QColor silkColor = ImageProcessor::getSilkColor(p.maskColorName);
        const QColor metalEdgeColor = ImageProcessor::getMetalRenderColor(p.finishType);

        if (p.useMetalEdge) {
            const QRgb metalRgb = metalEdgeColor.rgb();
            const QColor maskColor = ImageProcessor::getSolderMaskColor(p.maskColorName);
            const QRgb previewMaskLight = maskColor.lighter(135).rgb();
            for (int y = 0; y < h; ++y) {
                const uchar* em = (const uchar*)edgeMask.constScanLine(y);
                QRgb* lc = (QRgb*)res.copper.scanLine(y);
                QRgb* lm = (QRgb*)res.mask.scanLine(y);
                QRgb* ls = (QRgb*)res.silk.scanLine(y);
                QRgb* lp = (QRgb*)res.composite.scanLine(y);
                for (int x = 0; x < w; ++x) {
                    if (!em[x]) continue;
                    lc[x] = 0xFFFFFFFF;
                    if (p.exposeMetalEdge) {
                        ls[x] = 0xFF000000;
                        lm[x] = 0xFFFFFFFF;
                        lp[x] = metalRgb;
                    } else {
                        int bw = 255 - 170;
                        lp[x] = qRgb(
                            (qRed(lp[x])*bw   + qRed(previewMaskLight)*170)   / 255,
                            (qGreen(lp[x])*bw + qGreen(previewMaskLight)*170) / 255,
                            (qBlue(lp[x])*bw  + qBlue(previewMaskLight)*170)  / 255);
                    }
                }
            }
        } else {
            const QRgb silkRgb = silkColor.rgb();
            for (int y = 0; y < h; ++y) {
                const uchar* em = (const uchar*)edgeMask.constScanLine(y);
                QRgb* ls = (QRgb*)res.silk.scanLine(y);
                QRgb* lp = (QRgb*)res.composite.scanLine(y);
                for (int x = 0; x < w; ++x) {
                    if (!em[x]) continue;
                    ls[x] = 0xFFFFFFFF;
                    lp[x] = silkRgb;
                }
            }
        }
    }

    // 5. 生产层清理
    cleanBinaryLayer(res.copper, p.layerCleanupEnabled, p.layerCleanupMinArea);
    cleanBinaryLayer(res.mask,   p.layerCleanupEnabled, p.layerCleanupMinArea);
    cleanBinaryLayer(res.silk,   p.layerCleanupEnabled, p.layerCleanupMinArea);
    cleanBinaryLayer(res.bottom, p.layerCleanupEnabled, p.layerCleanupMinArea);

    // 6. 生成生产层（反色）
    auto invertImage = [](const QImage& img) {
        QImage inv = img.copy();
        inv.invertPixels();
        return inv;
    };
    res.layers["Top_Copper"] = invertImage(res.copper);
    res.layers["Top_Mask"]   = invertImage(res.mask);
    res.layers["Top_Silk"]   = invertImage(res.silk);
    res.layers["Bottom_Mask"]= invertImage(res.bottom);

    // 7. 灯光叠加（仅预览）
    if (p.showOverlay && !p.ledStrips.isEmpty()) {
        LEDLayoutEngine engine;
        engine.renderCompositeWithLEDs(
            res.composite, p.ledStrips, p.ledRadVal,
            res.copper, res.bottom,
            true, p.ledIntensity, true);
    }

    res.valid = true;
    return res;
}

// ============================================================
//  构造 / 析构
// ============================================================
MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent) {
    setupUI();
    setupMenuBar();
    initTempWorkspace();
    syncArgsToJson();

    // 防抖定时器：300ms 无操作后触发真正处理
    m_debounceTimer = new QTimer(this);
    m_debounceTimer->setSingleShot(true);
    m_debounceTimer->setInterval(300);
    connect(m_debounceTimer, &QTimer::timeout, this, &MainWindow::triggerProcess);

    // 异步处理完成监听
    m_watcher = new QFutureWatcher<ProcessResult>(this);
    connect(m_watcher, &QFutureWatcher<ProcessResult>::finished,
            this, &MainWindow::onProcessFinished);

    // 文件变化监听（画图实时更新）
    m_tempReloadTimer = new QTimer(this);
    connect(m_tempReloadTimer, &QTimer::timeout, this, &MainWindow::checkTempImageUpdated);
    m_tempReloadTimer->start(1200);

    setWindowTitle("PCB 透光画拆分工具 v1.6");
    statusBar()->showMessage("就绪 — 请通过 File > 导入图片 开始");
}

MainWindow::~MainWindow() {
    if (m_tempReloadTimer) m_tempReloadTimer->stop();
    if (m_watcher && m_watcher->isRunning()) {
        m_watcher->cancel();
        m_watcher->waitForFinished();
    }
    cleanupTempImages();
}

// ============================================================
//  UI 构建
// ============================================================

// IBM Carbon Gray100 Dark Theme — color tokens (only what's used directly in C++ code)
namespace CarbonDark {
    static const char* TEXT_SEC  = "#c6c6c6";  // Gray 30
    static const char* TEXT_HINT = "#8d8d8d";  // Gray 50
    static const char* BLUE40    = "#78a9ff";  // interactive on dark

    // Full app stylesheet
    static QString appStyle() {
        return QStringLiteral(
        // ----- Global -----
        "QWidget { background: #161616; color: #f4f4f4; font-family: 'Segoe UI', sans-serif; font-size: 15px; }"
        "QMainWindow { background: #161616; }"

        // ----- MenuBar -----
        "QMenuBar { background: #161616; color: #c6c6c6; border-bottom: 1px solid #393939; padding: 2px 0; font-size: 14px; }"
        "QMenuBar::item { padding: 6px 14px; border-radius: 0px; }"
        "QMenuBar::item:selected { background: #262626; color: #f4f4f4; }"
        "QMenu { background: #262626; color: #f4f4f4; border: 1px solid #393939; border-radius: 0px; padding: 4px 0; font-size: 14px; }"
        "QMenu::item { padding: 9px 28px 9px 16px; }"
        "QMenu::item:selected { background: #353535; color: #f4f4f4; }"
        "QMenu::separator { height: 1px; background: #393939; margin: 4px 0; }"

        // ----- ScrollArea -----
        "QScrollArea { border: none; background: transparent; }"
        "QScrollBar:vertical { background: #262626; width: 8px; border-radius: 4px; }"
        "QScrollBar::handle:vertical { background: #525252; border-radius: 4px; min-height: 24px; }"
        "QScrollBar::handle:vertical:hover { background: #6f6f6f; }"
        "QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }"

        // ----- GroupBox -----
        "QGroupBox { background: #262626; border: 1px solid #393939; border-radius: 0px;"
        "  margin-top: 10px; padding: 14px 8px 8px 8px; font-size: 14px; font-weight: 600; color: #c6c6c6; }"
        "QGroupBox::title { subcontrol-origin: margin; subcontrol-position: top left;"
        "  left: 8px; top: -1px; padding: 0 6px;"
        "  background: #262626; color: #c6c6c6; }"
        "QGroupBox::indicator { width: 16px; height: 16px; border: 1px solid #525252; border-radius: 0px; background: #161616; }"
        "QGroupBox::indicator:checked { background: #0f62fe; border-color: #0f62fe; image: none; }"
        "QGroupBox::indicator:hover { border-color: #78a9ff; }"

        // ----- Labels -----
        "QLabel { background: transparent; color: #f4f4f4; font-size: 15px; }"

        // ----- Sliders -----
        "QSlider::groove:horizontal { background: #393939; height: 2px; border-radius: 1px; }"
        "QSlider::handle:horizontal { background: #0f62fe; width: 16px; height: 16px; margin: -7px 0;"
        "  border-radius: 8px; border: 2px solid #0f62fe; }"
        "QSlider::handle:horizontal:hover { background: #78a9ff; border-color: #78a9ff; }"
        "QSlider::sub-page:horizontal { background: #0f62fe; height: 2px; border-radius: 1px; }"

        // ----- ComboBox -----
        "QComboBox { background: #393939; color: #f4f4f4; border: none; border-bottom: 2px solid #525252;"
        "  border-radius: 0px; padding: 8px 32px 8px 12px; min-height: 36px; font-size: 14px; }"
        "QComboBox:hover { border-bottom-color: #78a9ff; }"
        "QComboBox:focus { border-bottom-color: #0f62fe; }"
        "QComboBox::drop-down { border: none; width: 28px; }"
        "QComboBox::down-arrow { image: none; width: 0; height: 0;"
        "  border-left: 5px solid transparent; border-right: 5px solid transparent;"
        "  border-top: 6px solid #c6c6c6; margin-right: 8px; }"
        "QComboBox QAbstractItemView { background: #393939; color: #f4f4f4; border: 1px solid #525252;"
        "  selection-background-color: #0f62fe; outline: none; font-size: 14px; }"

        // ----- CheckBox -----
        "QCheckBox { color: #c6c6c6; spacing: 10px; font-size: 14px; }"
        "QCheckBox::indicator { width: 18px; height: 18px; border: 1px solid #525252; border-radius: 0px; background: #161616; }"
        "QCheckBox::indicator:checked { background: #0f62fe; border-color: #0f62fe; }"
        "QCheckBox::indicator:hover { border-color: #78a9ff; }"

        // ----- RadioButton -----
        "QRadioButton { color: #c6c6c6; spacing: 10px; font-size: 14px; }"
        "QRadioButton::indicator { width: 18px; height: 18px; border: 1px solid #525252; border-radius: 9px; background: #161616; }"
        "QRadioButton::indicator:checked { background: #0f62fe; border-color: #0f62fe; }"
        "QRadioButton::indicator:hover { border-color: #78a9ff; }"

        // ----- PushButton -----
        "QPushButton { background: #393939; color: #f4f4f4; border: none; border-radius: 0px;"
        "  padding: 0 16px; min-height: 44px; font-size: 15px; font-weight: 400; }"
        "QPushButton:hover { background: #4c4c4c; }"
        "QPushButton:pressed { background: #6f6f6f; }"
        "QPushButton:disabled { background: #262626; color: #525252; }"

        // ----- ProgressBar -----
        "QProgressBar { border: none; background: #262626; border-radius: 0px; max-height: 4px; }"
        "QProgressBar::chunk { background: #0f62fe; }"

        // ----- StatusBar -----
        "QStatusBar { background: #161616; color: #8d8d8d; border-top: 1px solid #393939; font-size: 13px; }"
        "QStatusBar::item { border: none; }"

        // ----- Tooltip -----
        "QToolTip { background: #393939; color: #f4f4f4; border: 1px solid #525252; padding: 6px 10px; border-radius: 0px; font-size: 13px; }"

        // ----- Dialog -----
        "QDialog { background: #262626; }"
        "QDialogButtonBox QPushButton { min-width: 88px; }"

        // ----- SpinBox -----
        "QSpinBox, QDoubleSpinBox { background: #393939; color: #f4f4f4; border: none;"
        "  border-bottom: 2px solid #525252; border-radius: 0px; padding: 4px 8px; min-height: 32px; }"
        "QSpinBox:focus, QDoubleSpinBox:focus { border-bottom-color: #0f62fe; }"
        "QSpinBox::up-button, QSpinBox::down-button, QDoubleSpinBox::up-button, QDoubleSpinBox::down-button"
        "  { background: #4c4c4c; border: none; width: 20px; }"
        );
    }
}

QSlider* MainWindow::createSlider(const QString& title, int min, int max, int def, QVBoxLayout* layout) {
    // Row: label left, value badge right
    QWidget* row = new QWidget;
    row->setStyleSheet("QWidget { background: transparent; }");
    QHBoxLayout* hl = new QHBoxLayout(row);
    hl->setContentsMargins(0, 0, 0, 0);
    hl->setSpacing(8);

    QLabel* titleLbl = new QLabel(title);
    titleLbl->setStyleSheet(QString("color: %1; font-size: 14px;")
                            .arg(CarbonDark::TEXT_SEC));
    titleLbl->setWordWrap(false);

    QLabel* valLbl = new QLabel(QString::number(def));
    valLbl->setStyleSheet(QString("color: %1; font-size: 14px; font-weight: 600;"
                                  " background: #393939; padding: 1px 6px; min-width: 36px;")
                          .arg(CarbonDark::BLUE40));
    valLbl->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    valLbl->setFixedWidth(44);

    hl->addWidget(titleLbl, 1);
    hl->addWidget(valLbl, 0);

    QSlider* s = new QSlider(Qt::Horizontal);
    s->setRange(min, max);
    s->setValue(def);

    layout->addWidget(row);
    layout->addWidget(s);

    connect(s, &QSlider::valueChanged, [=](int v) {
        valLbl->setText(QString::number(v));
        scheduleProcess();
    });
    return s;
}

QGroupBox* MainWindow::createCollapsibleGroup(const QString& title, QWidget* content, bool collapsed) {
    QGroupBox* box = new QGroupBox(title);
    box->setCheckable(true);
    box->setChecked(!collapsed);
    QVBoxLayout* lay = new QVBoxLayout(box);
    lay->setContentsMargins(8, 8, 8, 8);
    lay->setSpacing(8);
    lay->addWidget(content);
    content->setVisible(!collapsed);
    connect(box, &QGroupBox::toggled, content, &QWidget::setVisible);
    return box;
}

void MainWindow::setupUI() {
    // IBM Carbon Gray100 Dark Theme
    qApp->setStyle("Fusion");
    qApp->setStyleSheet(CarbonDark::appStyle());

    // Palette — needed so native widgets (scrollbars, etc.) also go dark
    QPalette dark;
    dark.setColor(QPalette::Window,          QColor(0x16, 0x16, 0x16));
    dark.setColor(QPalette::WindowText,      QColor(0xf4, 0xf4, 0xf4));
    dark.setColor(QPalette::Base,            QColor(0x26, 0x26, 0x26));
    dark.setColor(QPalette::AlternateBase,   QColor(0x39, 0x39, 0x39));
    dark.setColor(QPalette::Text,            QColor(0xf4, 0xf4, 0xf4));
    dark.setColor(QPalette::Button,          QColor(0x39, 0x39, 0x39));
    dark.setColor(QPalette::ButtonText,      QColor(0xf4, 0xf4, 0xf4));
    dark.setColor(QPalette::Highlight,       QColor(0x0f, 0x62, 0xfe));
    dark.setColor(QPalette::HighlightedText, Qt::white);
    dark.setColor(QPalette::ToolTipBase,     QColor(0x39, 0x39, 0x39));
    dark.setColor(QPalette::ToolTipText,     QColor(0xf4, 0xf4, 0xf4));
    dark.setColor(QPalette::Mid,             QColor(0x52, 0x52, 0x52));
    dark.setColor(QPalette::Dark,            QColor(0x16, 0x16, 0x16));
    dark.setColor(QPalette::Shadow,          QColor(0x00, 0x00, 0x00));
    qApp->setPalette(dark);

    QWidget* central = new QWidget;
    QHBoxLayout* mainLayout = new QHBoxLayout(central);
    mainLayout->setContentsMargins(0, 0, 0, 0);
    mainLayout->setSpacing(0);

    // ─── 左侧：主预览 ─────────────────────────────────────────
    QWidget* leftWidget = new QWidget;
    leftWidget->setStyleSheet("background: #161616;");
    QVBoxLayout* leftLayout = new QVBoxLayout(leftWidget);
    leftLayout->setContentsMargins(0, 0, 0, 0);
    leftLayout->setSpacing(0);

    // Top bar — title strip
    QWidget* previewBar = new QWidget;
    previewBar->setFixedHeight(40);
    previewBar->setStyleSheet("background: #262626; border-bottom: 1px solid #393939;");
    QHBoxLayout* pbLay = new QHBoxLayout(previewBar);
    pbLay->setContentsMargins(16, 0, 16, 0);
    pbLay->setSpacing(16);
    QLabel* previewTitle = new QLabel("预览");
    previewTitle->setStyleSheet("color: #f4f4f4; font-size: 13px; font-weight: 600;");
    QLabel* previewHint = new QLabel("滚轮缩放  ·  右键拖动  ·  左键布灯");
    previewHint->setStyleSheet(QString("color: %1; font-size: 12px; letter-spacing: 0.16px;")
                               .arg(CarbonDark::TEXT_HINT));
    pbLay->addWidget(previewTitle);
    pbLay->addWidget(previewHint);
    pbLay->addStretch();

    l_composite = new QLabel("导入图片开始");
    l_composite->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
    l_composite->setMinimumSize(480, 480);
    l_composite->setAlignment(Qt::AlignCenter);
    l_composite->setStyleSheet(
        "border: none;"
        "background: #0d0d0d;"
        "color: #525252;"
        "font-size: 14px;"
        "letter-spacing: 0.16px;");
    l_composite->installEventFilter(this);

    m_progressBar = new QProgressBar;
    m_progressBar->setRange(0, 0);
    m_progressBar->setVisible(false);
    m_progressBar->setMaximumHeight(4);
    m_progressBar->setTextVisible(false);

    leftLayout->addWidget(previewBar);
    leftLayout->addWidget(l_composite, 1);
    leftLayout->addWidget(m_progressBar);

    // ─── 中间：控制面板（Carbon 风格，带分隔线，带滚动条）────
    QWidget* ctrlWidget = new QWidget;
    ctrlWidget->setStyleSheet("background: #161616;");
    QVBoxLayout* ctrl = new QVBoxLayout(ctrlWidget);
    ctrl->setContentsMargins(0, 0, 0, 0);
    ctrl->setSpacing(0);

    // Helper: 带左边框的区段标题
    auto makeSection = [](const QString& text) {
        QLabel* lbl = new QLabel(text);
        lbl->setStyleSheet(
            "background: #262626;"
            "color: #f4f4f4;"
            "font-size: 14px;"
            "font-weight: 600;"
            "padding: 10px 16px;"
            "border-bottom: 1px solid #393939;");
        return lbl;
    };

    // Helper: 标准内容容器
    auto makeContentWidget = [](QVBoxLayout*& lay) {
        QWidget* w = new QWidget;
        w->setStyleSheet("background: #161616;");
        lay = new QVBoxLayout(w);
        lay->setContentsMargins(16, 12, 16, 12);
        lay->setSpacing(8);
        return w;
    };

    // Helper: 字段标签
    auto makeFieldLabel = [](const QString& text) {
        QLabel* l = new QLabel(text);
        l->setStyleSheet("color: #8d8d8d; font-size: 13px; background: transparent;");
        return l;
    };

    // --- 基础参数 ---
    ctrl->addWidget(makeSection("基础参数"));
    QVBoxLayout* basicLay;
    ctrl->addWidget(makeContentWidget(basicLay));
    basicLay->setSpacing(8);

    basicLay->addWidget(makeFieldLabel("表面处理工艺"));
    combo_surfaceFinish = new QComboBox;
    combo_surfaceFinish->addItems({"沉金 (ENIG, 金色)", "喷锡 (HASL, 银色)"});
    basicLay->addWidget(combo_surfaceFinish);

    basicLay->addWidget(makeFieldLabel("阻焊颜色"));
    combo_maskColor = new QComboBox;
    combo_maskColor->addItems({"蓝色", "黑色", "红色", "绿色"});
    basicLay->addWidget(combo_maskColor);

    // 分隔
    QFrame* sep1 = new QFrame; sep1->setFrameShape(QFrame::HLine);
    sep1->setStyleSheet("color: #393939; background: #393939; max-height: 1px;");
    basicLay->addWidget(sep1);

    s_gold       = createSlider("金属色相阈值 (沉金≈45°, 喷锡用亮度)", 0, 359, 45, basicLay);
    s_silk       = createSlider("丝印亮度阈值", 0, 255, 180, basicLay);
    s_trans      = createSlider("底层透光阈值", 0, 255, 120, basicLay);
    s_copperDepth= createSlider("敷铜遮掩阈值", 0, 255, 150, basicLay);

    // --- 图像预处理（减噪 + 色块整理，PCB制图必备）---
    ctrl->addWidget(makeSection("图像预处理"));
    QWidget* prepWidget = new QWidget;
    prepWidget->setStyleSheet("background: #161616;");
    QVBoxLayout* prepLay = new QVBoxLayout(prepWidget);
    prepLay->setContentsMargins(16, 12, 16, 12);
    prepLay->setSpacing(10);

    // 说明文字
    QLabel* prepHint = new QLabel("降低噪点，合并细碎色块，让颜色分层更干净。\n建议 PCB 制图时开启。");
    prepHint->setWordWrap(true);
    prepHint->setStyleSheet("color: #8d8d8d; font-size: 13px; background: transparent;");
    prepLay->addWidget(prepHint);

    // 高斯去噪开关 + 强度
    QCheckBox* chk_denoise = new QCheckBox("开启去噪（高斯模糊）");
    chk_denoise->setChecked(m_photoPreprocessEnabled);
    prepLay->addWidget(chk_denoise);

    // 去噪强度滑条 (核大小 3~11 奇数，值越大越模糊)
    QWidget* denoiseRow = new QWidget;
    denoiseRow->setStyleSheet("background: transparent;");
    QVBoxLayout* denoiseLay = new QVBoxLayout(denoiseRow);
    denoiseLay->setContentsMargins(0,0,0,0);
    denoiseLay->setSpacing(4);

    QWidget* dnLblRow = new QWidget; dnLblRow->setStyleSheet("background:transparent;");
    QHBoxLayout* dnLblLay = new QHBoxLayout(dnLblRow);
    dnLblLay->setContentsMargins(0,0,0,0);
    QLabel* dnTitle = new QLabel("去噪强度");
    dnTitle->setStyleSheet("color: #c6c6c6; font-size: 14px; background: transparent;");
    QLabel* dnVal = new QLabel(QString::number(m_photoPreprocessKernelSize));
    dnVal->setStyleSheet("color: #78a9ff; font-size: 14px; font-weight: 600; background: #393939; padding: 1px 6px; min-width: 36px;");
    dnVal->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    dnVal->setFixedWidth(44);
    dnLblLay->addWidget(dnTitle, 1);
    dnLblLay->addWidget(dnVal);
    denoiseLay->addWidget(dnLblRow);

    QSlider* sld_denoise = new QSlider(Qt::Horizontal);
    sld_denoise->setRange(1, 5);  // maps to kernel 3,5,7,9,11
    sld_denoise->setValue((m_photoPreprocessKernelSize - 1) / 2);
    denoiseLay->addWidget(sld_denoise);

    QLabel* dnHint = new QLabel("弱 ←——————→ 强");
    dnHint->setStyleSheet("color: #525252; font-size: 13px; background: transparent;");
    dnHint->setAlignment(Qt::AlignCenter);
    denoiseLay->addWidget(dnHint);
    prepLay->addWidget(denoiseRow);

    connect(sld_denoise, &QSlider::valueChanged, [=](int v) {
        int kernel = v * 2 + 1;  // 1->3, 2->5, 3->7, 4->9, 5->11
        dnVal->setText(QString::number(kernel));
        m_photoPreprocessKernelSize = kernel;
        if (chk_denoise->isChecked()) scheduleProcess();
    });

    // 颜色量化（色块整理）
    QCheckBox* chk_poster = new QCheckBox("开启颜色量化（色块整理）");
    chk_poster->setChecked(m_photoPreprocessEnabled);
    prepLay->addWidget(chk_poster);

    QWidget* posterRow = new QWidget; posterRow->setStyleSheet("background:transparent;");
    QVBoxLayout* posterLay = new QVBoxLayout(posterRow);
    posterLay->setContentsMargins(0,0,0,0); posterLay->setSpacing(4);

    QWidget* psLblRow = new QWidget; psLblRow->setStyleSheet("background:transparent;");
    QHBoxLayout* psLblLay = new QHBoxLayout(psLblRow);
    psLblLay->setContentsMargins(0,0,0,0);
    QLabel* psTitle = new QLabel("色块数量");
    psTitle->setStyleSheet("color: #c6c6c6; font-size: 14px; background: transparent;");
    QLabel* psVal = new QLabel(QString::number(m_photoPosterizeLevels));
    psVal->setStyleSheet("color: #78a9ff; font-size: 14px; font-weight: 600; background: #393939; padding: 1px 6px; min-width: 36px;");
    psVal->setAlignment(Qt::AlignRight | Qt::AlignVCenter);
    psVal->setFixedWidth(44);
    psLblLay->addWidget(psTitle, 1);
    psLblLay->addWidget(psVal);
    posterLay->addWidget(psLblRow);

    QSlider* sld_poster = new QSlider(Qt::Horizontal);
    sld_poster->setRange(2, 32);
    sld_poster->setValue(qBound(2, m_photoPosterizeLevels, 32));
    posterLay->addWidget(sld_poster);

    QLabel* psHint = new QLabel("少（色块大）←———→ 多（细节多）");
    psHint->setStyleSheet("color: #525252; font-size: 13px; background: transparent;");
    psHint->setAlignment(Qt::AlignCenter);
    posterLay->addWidget(psHint);
    prepLay->addWidget(posterRow);

    // 同步 enable 状态
    auto syncPrepState = [=]() {
        bool dn = chk_denoise->isChecked();
        bool ps = chk_poster->isChecked();
        denoiseRow->setEnabled(dn);
        posterRow->setEnabled(ps);
        m_denoiseEnabled  = dn;
        m_posterizeEnabled = ps;
        m_photoPreprocessEnabled = dn || ps;
        scheduleProcess();
    };
    connect(chk_denoise, &QCheckBox::toggled, [=](bool) { syncPrepState(); });
    connect(chk_poster,  &QCheckBox::toggled, [=](bool) { syncPrepState(); });
    connect(sld_poster,  &QSlider::valueChanged, [=](int v) {
        psVal->setText(QString::number(v));
        m_photoPosterizeLevels = v;
        if (chk_poster->isChecked()) scheduleProcess();
    });
    syncPrepState();
    ctrl->addWidget(prepWidget);

    // --- 灯光 ---
    QWidget* lightContent = new QWidget;
    lightContent->setStyleSheet("background: #161616;");
    QVBoxLayout* lightLay = new QVBoxLayout(lightContent);
    lightLay->setContentsMargins(16, 12, 16, 12);
    lightLay->setSpacing(8);

    check_showLEDOverlay = new QCheckBox("预览中叠加显示灯光范围");
    connect(check_showLEDOverlay, &QCheckBox::toggled, this, &MainWindow::scheduleProcess);
    lightLay->addWidget(check_showLEDOverlay);

    s_autoSense = createSlider("自动布灯数量 (0=关闭)", 0, 10, 1, lightLay);

    QHBoxLayout* ledBtnRow = new QHBoxLayout;
    ledBtnRow->setSpacing(8);
    QPushButton* btn_auto = new QPushButton("自动布灯");
    btn_auto->setToolTip("根据图像颜色分布自动放置灯条");
    btn_auto->setStyleSheet(
        "QPushButton { background: #0f62fe; color: #fff; font-weight: 600; border: none; min-height: 40px; }"
        "QPushButton:hover { background: #0353e9; }"
        "QPushButton:pressed { background: #002d9c; }");
    connect(btn_auto, &QPushButton::clicked, this, &MainWindow::autoSuggestLEDs);
    ledBtnRow->addWidget(btn_auto);

    QPushButton* btn_clearLED = new QPushButton("清除灯条");
    btn_clearLED->setStyleSheet(
        "QPushButton { background: transparent; color: #da1e28; border: 1px solid #da1e28; min-height: 40px; }"
        "QPushButton:hover { background: #2d0a0b; }"
        "QPushButton:pressed { background: #520408; }");
    connect(btn_clearLED, &QPushButton::clicked, this, &MainWindow::clearAllLEDs);
    ledBtnRow->addWidget(btn_clearLED);
    lightLay->addLayout(ledBtnRow);

    s_ledRad      = createSlider("散射半径 (像素)", 20, 500, 150, lightLay);
    s_ledIntensity= createSlider("中心不透明度",    0,  255, 200, lightLay);

    check_lightEnable = new QCheckBox("启用灯光效果");
    check_lightEnable->setChecked(false);
    QGroupBox* grpLight = createCollapsibleGroup("灯光 / 布灯", lightContent, true);
    ctrl->addWidget(makeSection("灯光 / LED"));
    ctrl->addWidget(grpLight);
    connect(check_lightEnable, &QCheckBox::toggled, grpLight, &QGroupBox::setChecked);
    connect(grpLight, &QGroupBox::toggled, [=](bool on){
        check_lightEnable->blockSignals(true);
        check_lightEnable->setChecked(on);
        check_lightEnable->blockSignals(false);
        scheduleProcess();
    });

    // --- 裸露基材 ---
    QWidget* bareContent = new QWidget;
    bareContent->setStyleSheet("background: #161616;");
    QVBoxLayout* bareLay = new QVBoxLayout(bareContent);
    bareLay->setContentsMargins(16, 12, 16, 12);
    bareLay->setSpacing(8);

    QButtonGroup* bareModeGrp = new QButtonGroup(bareContent);
    bareModeGrp->setExclusive(true);
    radio_bareSubstrateGray  = new QRadioButton("按灰度绑定");
    radio_bareSubstrateColor = new QRadioButton("按颜色相似度绑定");
    bareModeGrp->addButton(radio_bareSubstrateGray);
    bareModeGrp->addButton(radio_bareSubstrateColor);
    radio_bareSubstrateGray->setChecked(true);
    bareLay->addWidget(radio_bareSubstrateGray);
    s_bareSubstrateGrayA = createSlider("灰度下限 A (%)", 0, 100, 20, bareLay);
    s_bareSubstrateGrayB = createSlider("灰度上限 B (%)", 0, 100, 65, bareLay);
    bareLay->addWidget(radio_bareSubstrateColor);
    s_bareSubstrateColorSimilarity = createSlider("颜色相似度 C (%)", 0, 100, 80, bareLay);

    check_bareSubstrateEnable = new QCheckBox("启用裸露基材");
    QGroupBox* grpBare = createCollapsibleGroup("裸露基材绑定", bareContent, true);
    m_grpBare = grpBare;
    ctrl->addWidget(makeSection("裸露基材"));
    ctrl->addWidget(grpBare);

    auto syncBareState = [=]() {
        bool on = grpBare->isChecked();
        bool grayMode = radio_bareSubstrateGray->isChecked();
        s_bareSubstrateGrayA->setEnabled(on && grayMode);
        s_bareSubstrateGrayB->setEnabled(on && grayMode);
        s_bareSubstrateColorSimilarity->setEnabled(on && !grayMode);
    };
    connect(grpBare, &QGroupBox::toggled, [=](bool){ syncBareState(); scheduleProcess(); });
    connect(radio_bareSubstrateGray,  &QRadioButton::toggled, [=](bool){ syncBareState(); scheduleProcess(); });
    connect(radio_bareSubstrateColor, &QRadioButton::toggled, [=](bool){ syncBareState(); scheduleProcess(); });
    syncBareState();

    // --- 边缘操作 ---
    QWidget* edgeContent = new QWidget;
    edgeContent->setStyleSheet("background: #161616;");
    QVBoxLayout* edgeLay = new QVBoxLayout(edgeContent);
    edgeLay->setContentsMargins(16, 12, 16, 12);
    edgeLay->setSpacing(8);

    QButtonGroup* edgeModeGrp = new QButtonGroup(edgeContent);
    edgeModeGrp->setExclusive(true);
    radio_edgeStroke  = new QRadioButton("Canny 描边");
    radio_edgeEnhance = new QRadioButton("拉普拉斯边缘增强");
    edgeModeGrp->addButton(radio_edgeStroke);
    edgeModeGrp->addButton(radio_edgeEnhance);
    radio_edgeEnhance->setChecked(true);
    edgeLay->addWidget(radio_edgeStroke);
    edgeLay->addWidget(radio_edgeEnhance);

    s_edgeThresh    = createSlider("边缘下限阈值",   0, 255,  50, edgeLay);
    s_edgeThreshMax = createSlider("边缘上限阈值",   0, 255, 200, edgeLay);

    check_useMetalEdge   = new QCheckBox("使用金属色勾线");
    check_exposeMetalEdge= new QCheckBox("裸露金属勾线（阻焊开窗）");
    check_exposeMetalEdge->setChecked(true);
    check_exposeMetalEdge->setVisible(false);
    edgeLay->addWidget(check_useMetalEdge);
    edgeLay->addWidget(check_exposeMetalEdge);

    s_autoInvert = createSlider("自动反色范围 (-1=强制黑, 0=强制白)", -1, 50, 10, edgeLay);

    check_edgeEnable = new QCheckBox("启用边缘操作");
    QGroupBox* grpEdge = createCollapsibleGroup("边缘操作", edgeContent, true);
    group_edgeOperation = grpEdge;
    ctrl->addWidget(makeSection("边缘操作"));
    ctrl->addWidget(grpEdge);

    auto syncEdgeState = [=]() {
        bool on = grpEdge->isChecked();
        bool useMetal = on && check_useMetalEdge->isChecked();
        s_autoInvert->setEnabled(!useMetal);
        check_exposeMetalEdge->setVisible(useMetal);
    };
    connect(grpEdge, &QGroupBox::toggled, [=](bool){ syncEdgeState(); scheduleProcess(); });
    connect(radio_edgeStroke,  &QRadioButton::toggled, [=](bool checked){ if(checked) scheduleProcess(); });
    connect(radio_edgeEnhance, &QRadioButton::toggled, [=](bool checked){ if(checked) scheduleProcess(); });
    connect(check_useMetalEdge,    &QCheckBox::toggled, [=](bool){ syncEdgeState(); scheduleProcess(); });
    connect(check_exposeMetalEdge, &QCheckBox::toggled, [=](bool){ scheduleProcess(); });
    syncEdgeState();

    // --- 操作按钮 ---
    ctrl->addWidget(makeSection("图纸操作"));
    QWidget* actWidget = new QWidget;
    actWidget->setStyleSheet("background: #161616;");
    QVBoxLayout* actLay = new QVBoxLayout(actWidget);
    actLay->setContentsMargins(16, 12, 16, 12);
    actLay->setSpacing(8);

    QPushButton* btn_import = new QPushButton("导入图片");
    btn_import->setStyleSheet(
        "QPushButton { background: #0f62fe; color: #fff; font-weight: 600; border: none; min-height: 48px; font-size: 14px; }"
        "QPushButton:hover { background: #0353e9; }"
        "QPushButton:pressed { background: #002d9c; }");
    connect(btn_import, &QPushButton::clicked, this, &MainWindow::loadAndProcess);

    btn_export = new QPushButton("导出生产层");
    btn_export->setEnabled(false);
    btn_export->setStyleSheet(
        "QPushButton { background: #393939; color: #f4f4f4; font-weight: 600; border: none; min-height: 48px; font-size: 14px; }"
        "QPushButton:hover { background: #4c4c4c; }"
        "QPushButton:pressed { background: #6f6f6f; }"
        "QPushButton:disabled { background: #262626; color: #525252; }");
    connect(btn_export, &QPushButton::clicked, this, &MainWindow::exportLayers);

    actLay->addWidget(btn_import);
    actLay->addWidget(btn_export);

    check_expandPreviews = new QCheckBox("展开右侧生产层预览");
    actLay->addWidget(check_expandPreviews);
    ctrl->addWidget(actWidget);
    ctrl->addStretch();

    // 将控制面板放入 QScrollArea
    QScrollArea* scroll = new QScrollArea;
    scroll->setWidget(ctrlWidget);
    scroll->setWidgetResizable(true);
    scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
    scroll->setFixedWidth(300);
    scroll->setStyleSheet(
        "QScrollArea { border: none; border-left: 1px solid #393939; background: #161616; }"
        "QScrollArea > QWidget > QWidget { background: #161616; }");

    // 连接 combo 信号
    connect(combo_surfaceFinish, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::scheduleProcess);
    connect(combo_maskColor, QOverload<int>::of(&QComboBox::currentIndexChanged),
            this, &MainWindow::scheduleProcess);

    // ─── 右侧：四层预览 ──────────────────────────────────────
    QWidget* rightWidget = new QWidget;
    rightWidget->setStyleSheet("background: #161616; border-left: 1px solid #393939;");
    QVBoxLayout* rightOuter = new QVBoxLayout(rightWidget);
    rightOuter->setContentsMargins(0, 0, 0, 0);
    rightOuter->setSpacing(0);

    // Header
    QWidget* rightHeader = new QWidget;
    rightHeader->setFixedHeight(40);
    rightHeader->setStyleSheet("background: #262626; border-bottom: 1px solid #393939;");
    QHBoxLayout* rhLay = new QHBoxLayout(rightHeader);
    rhLay->setContentsMargins(16, 0, 16, 0);
    QLabel* rightTitle = new QLabel("生产层预览");
    rightTitle->setStyleSheet("color: #f4f4f4; font-size: 13px; font-weight: 600;");
    rhLay->addWidget(rightTitle);
    rhLay->addStretch();
    rightOuter->addWidget(rightHeader);

    QGridLayout* rightGrid = new QGridLayout;
    rightGrid->setSpacing(1);
    rightGrid->setContentsMargins(8, 8, 8, 8);

    auto makeLayerLabel = [&](const QString& title, QLabel*& lbl) {
        lbl = new QLabel;
        lbl->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
        lbl->setMinimumSize(200, 190);
        lbl->setAlignment(Qt::AlignCenter);
        lbl->setStyleSheet("border: none; background: #0d0d0d;");
        return lbl;
    };

    rightGrid->addWidget(makeLayerLabel("线路层\nTop Copper",  l_copper),  0, 0);
    rightGrid->addWidget(makeLayerLabel("阻焊层\nTop Mask",    l_mask),    0, 1);
    rightGrid->addWidget(makeLayerLabel("丝印层\nTop Silk",    l_silk),    1, 0);
    rightGrid->addWidget(makeLayerLabel("透光层\nBottom Mask", l_bottom),  1, 1);

    m_layerPreviewKeys[l_copper]  = "Top_Copper";
    m_layerPreviewKeys[l_mask]    = "Top_Mask";
    m_layerPreviewKeys[l_silk]    = "Top_Silk";
    m_layerPreviewKeys[l_bottom]  = "Bottom_Mask";
    m_layerPreviewTitles[l_copper] = "线路层  Top Copper";
    m_layerPreviewTitles[l_mask]   = "阻焊层  Top Mask";
    m_layerPreviewTitles[l_silk]   = "丝印层  Top Silk";
    m_layerPreviewTitles[l_bottom] = "透光层  Bottom Mask";

    l_copper->installEventFilter(this);
    l_mask->installEventFilter(this);
    l_silk->installEventFilter(this);
    l_bottom->installEventFilter(this);

    rightOuter->addLayout(rightGrid, 1);

    rightWidget->setVisible(false);
    connect(check_expandPreviews, &QCheckBox::toggled, rightWidget, &QWidget::setVisible);

    // ─── 整体布局 ─────────────────────────────────────────────
    mainLayout->addWidget(leftWidget, 5);
    mainLayout->addWidget(scroll,     0);
    mainLayout->addWidget(rightWidget,3);

    setCentralWidget(central);
    resize(1360, 860);

    // 状态栏
    m_statusLabel = new QLabel("就绪");
    m_statusLabel->setStyleSheet(QString("color: %1; font-size: 12px;").arg(CarbonDark::TEXT_HINT));
    statusBar()->addWidget(m_statusLabel, 1);
}

void MainWindow::setupMenuBar() {
    QMenu* fileMenu = menuBar()->addMenu("文件");
    fileMenu->addAction("导入图片",   this, &MainWindow::loadAndProcess, QKeySequence::Open);
    action_exportLayers = fileMenu->addAction("导出生产层", this, &MainWindow::exportLayers);
    action_exportLayers->setEnabled(false);
    action_exportLayers->setShortcut(QKeySequence("Ctrl+E"));
    fileMenu->addSeparator();
    fileMenu->addAction("保存工程 (.pcblg)", this, &MainWindow::saveProject,   QKeySequence::Save);
    fileMenu->addAction("导入工程 (.pcblg)", this, &MainWindow::importProject, QKeySequence("Ctrl+Shift+O"));
    fileMenu->addSeparator();
    fileMenu->addAction("画图实时编辑",  this, &MainWindow::openPaintEditor);

    QMenu* optMenu = menuBar()->addMenu("选项");
    optMenu->addAction("边缘滤波预处理",    this, &MainWindow::openFilterPreprocessDialog);
    optMenu->addAction("道格拉斯-普克抽稀", this, &MainWindow::openDouglasPeuckerDialog);
}

// ============================================================
//  防抖 + 异步处理
// ============================================================
void MainWindow::scheduleProcess() {
    if (m_debounceTimer) m_debounceTimer->start(); // 重置计时器
}

void MainWindow::triggerProcess() {
    if (m_origin.isNull()) {
        syncArgsToJson();
        return;
    }
    if (m_processingInFlight) {
        m_pendingRequest = true;  // 记下来，处理完成后再触发一次
        return;
    }
    setProcessingState(true);
    ProcessParams p = collectParams();
    QFuture<ProcessResult> future = QtConcurrent::run(MainWindow::runProcessing, p);
    m_watcher->setFuture(future);
}

void MainWindow::onProcessFinished() {
    ProcessResult res = m_watcher->result();
    setProcessingState(false);
    if (res.valid) {
        applyLayerResults(res);
        // Show completion info in status bar
        const QString msg = QString("处理完成  %1 × %2 px")
            .arg(res.composite.width())
            .arg(res.composite.height());
        if (m_statusLabel) m_statusLabel->setText(msg);
        // Brief highlight animation via timer reset
        QTimer::singleShot(4000, this, [this](){
            if (m_statusLabel && !m_processingInFlight)
                m_statusLabel->setText("就绪");
        });
    } else {
        if (m_statusLabel) m_statusLabel->setText("处理失败，请检查图片格式");
    }
    if (!m_isApplyingArgs) syncArgsToJson();
    if (m_pendingRequest) {
        m_pendingRequest = false;
        QTimer::singleShot(0, this, &MainWindow::triggerProcess);
    }
}

void MainWindow::setProcessingState(bool processing) {
    m_processingInFlight = processing;
    if (m_progressBar) m_progressBar->setVisible(processing);
    if (m_statusLabel) {
        m_statusLabel->setText(processing ? "处理中..." : "就绪");
    }
}

MainWindow::ProcessParams MainWindow::collectParams() const {
    ProcessParams p;
    p.srcImage = m_origin;

    p.goldThresh   = s_gold       ? s_gold->value()        : 45;
    p.silkThresh   = s_silk       ? s_silk->value()        : 180;
    p.transThresh  = s_trans      ? s_trans->value()       : 120;
    p.copperDepth  = s_copperDepth? s_copperDepth->value() : 150;
    p.ledRadVal    = s_ledRad     ? s_ledRad->value()      : 150;
    p.ledIntensity = s_ledIntensity?s_ledIntensity->value(): 200;

    p.maskColorName = combo_maskColor    ? combo_maskColor->currentText()    : "蓝色";
    p.finishType    = combo_surfaceFinish? combo_surfaceFinish->currentText(): "沉金 (ENIG, 金色)";
    p.isWhiteMask   = false;

    p.enableBareSubstrate          = (m_grpBare && m_grpBare->isChecked());
    p.bareSubstrateUseGrayBinding  = radio_bareSubstrateGray  && radio_bareSubstrateGray->isChecked();
    p.bareSubstrateGrayMinPct      = s_bareSubstrateGrayA     ? s_bareSubstrateGrayA->value()     : 20;
    p.bareSubstrateGrayMaxPct      = s_bareSubstrateGrayB     ? s_bareSubstrateGrayB->value()     : 65;
    p.bareSubstrateColorSimilarityPct = s_bareSubstrateColorSimilarity ? s_bareSubstrateColorSimilarity->value() : 80;

    p.edgeEnabled  = group_edgeOperation && group_edgeOperation->isChecked();
    p.edgeMode     = (radio_edgeStroke && radio_edgeStroke->isChecked())
                        ? EdgeSharpener::OperationMode::StrokeCanny
                        : EdgeSharpener::OperationMode::EdgeEnhance;
    p.edgeThreshMin = s_edgeThresh    ? s_edgeThresh->value()    : 50;
    p.edgeThreshMax = s_edgeThreshMax ? s_edgeThreshMax->value() : 200;
    p.autoInvert    = s_autoInvert    ? s_autoInvert->value()    : 10;
    p.useMetalEdge   = check_useMetalEdge    && check_useMetalEdge->isChecked();
    p.exposeMetalEdge= check_exposeMetalEdge && check_exposeMetalEdge->isChecked();

    p.edgePrefilterEnabled   = m_edgePrefilterEnabled;
    p.edgePrefilterKernelSize= m_edgePrefilterKernelSize;
    p.edgePrefilterSigma     = m_edgePrefilterSigma;

    p.photoPreprocessEnabled   = m_denoiseEnabled || m_posterizeEnabled;
    p.denoiseEnabled           = m_denoiseEnabled;
    p.posterizeEnabled         = m_posterizeEnabled;
    p.photoPreprocessKernelSize= m_photoPreprocessKernelSize;
    p.photoPosterizeLevels     = m_photoPosterizeLevels;

    p.layerCleanupEnabled = m_layerCleanupEnabled;
    p.layerCleanupMinArea = m_layerCleanupMinArea;

    p.dpEnabled   = m_dpEnabled;
    p.dpTolerance = m_dpTolerance;
    p.dpLineWidth = m_dpLineWidth;

    p.showOverlay = (check_lightEnable && check_lightEnable->isChecked() &&
                     check_showLEDOverlay && check_showLEDOverlay->isChecked());
    p.ledStrips = m_ledStrips;

    return p;
}

void MainWindow::applyLayerResults(const ProcessResult& res) {
    m_layers = res.layers;
    m_previewComposite = res.composite;

    updateCompositePreview(m_previewComposite);
    updateLayerPreview(l_copper, m_layers.value("Top_Copper"), m_layerPreviewStates[l_copper]);
    updateLayerPreview(l_mask,   m_layers.value("Top_Mask"),   m_layerPreviewStates[l_mask]);
    updateLayerPreview(l_silk,   m_layers.value("Top_Silk"),   m_layerPreviewStates[l_silk]);
    updateLayerPreview(l_bottom, m_layers.value("Bottom_Mask"),m_layerPreviewStates[l_bottom]);
}

// ============================================================
//  预览渲染
// ============================================================
void MainWindow::clampPreviewPan(QLabel* label, const QImage& img, PreviewState& state) {
    if (!label || img.isNull() || label->size().isEmpty()) {
        state.pan = QPointF(0,0); return;
    }
    const QRectF r = calcPreviewRect(label->size(), img.size(), state.zoom, QPointF(0,0));
    const double maxX = qMax(0.0, (r.width()  - label->width())  * 0.5);
    const double maxY = qMax(0.0, (r.height() - label->height()) * 0.5);
    state.pan.setX(qBound(-maxX, state.pan.x(), maxX));
    state.pan.setY(qBound(-maxY, state.pan.y(), maxY));
}

void MainWindow::clampPreviewPan() {
    if (m_previewComposite.isNull() || !l_composite || l_composite->size().isEmpty()) {
        m_previewPan = QPointF(0,0); return;
    }
    const QRectF r = calcPreviewRect(l_composite->size(), m_previewComposite.size(), m_previewZoom, QPointF(0,0));
    const double maxX = qMax(0.0, (r.width()  - l_composite->width())  * 0.5);
    const double maxY = qMax(0.0, (r.height() - l_composite->height()) * 0.5);
    m_previewPan.setX(qBound(-maxX, m_previewPan.x(), maxX));
    m_previewPan.setY(qBound(-maxY, m_previewPan.y(), maxY));
}

void MainWindow::updateLayerPreview(QLabel* label, const QImage& img, PreviewState& state) {
    if (!label || img.isNull() || label->size().isEmpty()) return;
    clampPreviewPan(label, img, state);
    const QRectF targetRect = calcPreviewRect(label->size(), img.size(), state.zoom, state.pan);
    QPixmap canvas(label->size());
    canvas.fill(QColor(13, 13, 13));
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.drawImage(targetRect, img);

    // Overlay title — font size scales with label width
    const QString title = m_layerPreviewTitles.value(label);
    if (!title.isEmpty()) {
        const int w = label->width();
        const int fontSize = qBound(13, w / 14, 22);  // 自适应字号
        QFont f = painter.font();
        f.setPixelSize(fontSize);
        f.setBold(true);
        painter.setFont(f);

        // Shadow
        QRect textRect = canvas.rect().adjusted(0, 0, 0, -(canvas.height() - fontSize * 2));
        painter.setPen(QColor(0, 0, 0, 180));
        painter.drawText(textRect.adjusted(1, 1, 1, 1), Qt::AlignLeft | Qt::AlignTop, title);
        // Text
        painter.setPen(QColor(220, 220, 220, 230));
        painter.drawText(textRect, Qt::AlignLeft | Qt::AlignTop, title);
    }
    painter.end();
    label->setPixmap(canvas);
}

void MainWindow::updateCompositePreview(const QImage& img) {
    if (!l_composite || img.isNull() || l_composite->size().isEmpty()) return;
    clampPreviewPan();
    const QRectF targetRect = calcPreviewRect(l_composite->size(), img.size(), m_previewZoom, m_previewPan);
    QPixmap canvas(l_composite->size());
    canvas.fill(QColor(17, 17, 17));
    QPainter painter(&canvas);
    painter.setRenderHint(QPainter::SmoothPixmapTransform, true);
    painter.drawImage(targetRect, img);

    // 如果正在布灯第一点，画一个十字提示
    if (m_isPlacing) {
        painter.setPen(QPen(QColor(255, 200, 0, 180), 1, Qt::DashLine));
        QPoint lp;
        // 不映射鼠标位置，仅画简单说明
        painter.drawText(canvas.rect().adjusted(4,4,-4,-4), Qt::AlignBottom | Qt::AlignHCenter,
                         "已记录第一点，再次点击放置灯条");
    }
    painter.end();
    l_composite->setPixmap(canvas);
}

bool MainWindow::mapLabelToImage(const QPoint& labelPos, QPoint& imgPos) const {
    if (m_previewComposite.isNull() || !l_composite || l_composite->size().isEmpty()) return false;
    const QRectF drawRect = calcPreviewRect(l_composite->size(), m_previewComposite.size(), m_previewZoom, m_previewPan);
    if (!drawRect.contains(QPointF(labelPos))) return false;
    const double nx = (labelPos.x() - drawRect.left()) / drawRect.width();
    const double ny = (labelPos.y() - drawRect.top())  / drawRect.height();
    imgPos = QPoint(
        qBound(0, (int)(nx * m_previewComposite.width()),  m_previewComposite.width()  - 1),
        qBound(0, (int)(ny * m_previewComposite.height()), m_previewComposite.height() - 1));
    return true;
}

// ============================================================
//  事件过滤
// ============================================================
bool MainWindow::handleLayerPreviewEvent(QLabel* label, QEvent* event, const QImage& img, PreviewState& state) {
    if (!label || img.isNull()) return false;

    if (event->type() == QEvent::Wheel) {
        QWheelEvent* we = static_cast<QWheelEvent*>(event);
        const int delta = we->angleDelta().y();
        if (delta == 0) return true;
        const double factor = std::pow(1.12, delta / 120.0);
        const double oldZoom = state.zoom;
        const double newZoom = qBound(0.2, oldZoom * factor, 8.0);
        if (std::abs(newZoom - oldZoom) < 1e-6) return true;
        const QPoint cp = we->position().toPoint();
        const QRectF oldRect = calcPreviewRect(label->size(), img.size(), oldZoom, state.pan);
        state.zoom = newZoom;
        if (oldRect.contains(QPointF(cp))) {
            const double u = (cp.x() - oldRect.left()) / oldRect.width();
            const double v = (cp.y() - oldRect.top())  / oldRect.height();
            const QRectF nr = calcPreviewRect(label->size(), img.size(), state.zoom, state.pan);
            state.pan += QPointF(cp.x() - (nr.left() + u*nr.width()),
                                 cp.y() - (nr.top()  + v*nr.height()));
        }
        clampPreviewPan(label, img, state);
        updateLayerPreview(label, img, state);
        return true;
    }
    if (event->type() == QEvent::MouseButtonPress) {
        QMouseEvent* me = static_cast<QMouseEvent*>(event);
        if (me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) {
            state.isPanning = true;
            state.lastPanPos = me->pos();
            label->setCursor(Qt::ClosedHandCursor);
            return true;
        }
    }
    if (event->type() == QEvent::MouseMove && state.isPanning) {
        QMouseEvent* me = static_cast<QMouseEvent*>(event);
        const QPoint delta = me->pos() - state.lastPanPos;
        state.lastPanPos = me->pos();
        state.pan += QPointF(delta.x(), delta.y());
        clampPreviewPan(label, img, state);
        updateLayerPreview(label, img, state);
        return true;
    }
    if (event->type() == QEvent::MouseButtonRelease) {
        QMouseEvent* me = static_cast<QMouseEvent*>(event);
        if ((me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) && state.isPanning) {
            state.isPanning = false;
            label->unsetCursor();
            return true;
        }
    }
    return false;
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event) {
    // 主预览区
    if (obj == l_composite) {
        if (event->type() == QEvent::Wheel) {
            QWheelEvent* we = static_cast<QWheelEvent*>(event);
            const int delta = we->angleDelta().y();
            if (delta == 0 || m_previewComposite.isNull()) return true;
            const double factor = std::pow(1.12, delta / 120.0);
            const double oldZoom = m_previewZoom;
            const double newZoom = qBound(0.2, oldZoom * factor, 8.0);
            if (std::abs(newZoom - oldZoom) < 1e-6) return true;
            const QPoint cp = we->position().toPoint();
            const QRectF oldRect = calcPreviewRect(l_composite->size(), m_previewComposite.size(), oldZoom, m_previewPan);
            m_previewZoom = newZoom;
            if (oldRect.contains(QPointF(cp))) {
                const double u = (cp.x() - oldRect.left()) / oldRect.width();
                const double v = (cp.y() - oldRect.top())  / oldRect.height();
                const QRectF nr = calcPreviewRect(l_composite->size(), m_previewComposite.size(), m_previewZoom, m_previewPan);
                m_previewPan += QPointF(cp.x() - (nr.left() + u*nr.width()),
                                        cp.y() - (nr.top()  + v*nr.height()));
            }
            clampPreviewPan();
            updateCompositePreview(m_previewComposite);
            return true;
        }
        if (event->type() == QEvent::MouseButtonPress) {
            QMouseEvent* me = static_cast<QMouseEvent*>(event);
            if (me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) {
                m_isPanningPreview = true;
                m_lastPanPos = me->pos();
                l_composite->setCursor(Qt::ClosedHandCursor);
                return true;
            }
            if (me->button() == Qt::LeftButton && !m_previewComposite.isNull()) {
                QPoint imgPos;
                if (mapLabelToImage(me->pos(), imgPos)) {
                    if (!m_isPlacing) {
                        m_pendingStart = imgPos;
                        m_isPlacing = true;
                        m_statusLabel->setText(QString("布灯第一点: (%1, %2)  — 再次点击确认").arg(imgPos.x()).arg(imgPos.y()));
                        updateCompositePreview(m_previewComposite);
                    } else {
                        LEDStrip s;
                        s.start  = m_pendingStart;
                        s.end    = imgPos;
                        s.radius = s_ledRad ? s_ledRad->value() : 150;
                        // 颜色取终点像素，太暗则用白色
                        s.color  = m_origin.pixelColor(imgPos.x(), imgPos.y());
                        if (s.color.value() < 50) s.color = Qt::white;
                        m_ledStrips.append(s);
                        m_isPlacing = false;
                        m_statusLabel->setText(QString("已添加灯条 #%1，共 %2 条").arg(m_ledStrips.size()).arg(m_ledStrips.size()));
                        scheduleProcess();
                    }
                }
                return true;
            }
        }
        if (event->type() == QEvent::MouseMove && m_isPanningPreview) {
            QMouseEvent* me = static_cast<QMouseEvent*>(event);
            const QPoint delta = me->pos() - m_lastPanPos;
            m_lastPanPos = me->pos();
            m_previewPan += QPointF(delta.x(), delta.y());
            clampPreviewPan();
            updateCompositePreview(m_previewComposite);
            return true;
        }
        if (event->type() == QEvent::MouseButtonRelease) {
            QMouseEvent* me = static_cast<QMouseEvent*>(event);
            if ((me->button() == Qt::MiddleButton || me->button() == Qt::RightButton) && m_isPanningPreview) {
                m_isPanningPreview = false;
                l_composite->unsetCursor();
                return true;
            }
        }
    }

    // 四层预览区
    QLabel* lbl = qobject_cast<QLabel*>(obj);
    if (lbl && m_layerPreviewKeys.contains(lbl) && !m_layers.isEmpty()) {
        const QString key = m_layerPreviewKeys.value(lbl);
        if (m_layers.contains(key))
            return handleLayerPreviewEvent(lbl, event, m_layers.value(key), m_layerPreviewStates[lbl]);
    }
    return QMainWindow::eventFilter(obj, event);
}

void MainWindow::resizeEvent(QResizeEvent* event) {
    QMainWindow::resizeEvent(event);
    if (!m_previewComposite.isNull()) {
        clampPreviewPan();
        updateCompositePreview(m_previewComposite);
    }
    if (l_copper && m_layers.contains("Top_Copper"))
        updateLayerPreview(l_copper, m_layers["Top_Copper"], m_layerPreviewStates[l_copper]);
    if (l_mask && m_layers.contains("Top_Mask"))
        updateLayerPreview(l_mask,   m_layers["Top_Mask"],   m_layerPreviewStates[l_mask]);
    if (l_silk && m_layers.contains("Top_Silk"))
        updateLayerPreview(l_silk,   m_layers["Top_Silk"],   m_layerPreviewStates[l_silk]);
    if (l_bottom && m_layers.contains("Bottom_Mask"))
        updateLayerPreview(l_bottom, m_layers["Bottom_Mask"],m_layerPreviewStates[l_bottom]);
}

// ============================================================
//  自动布灯
// ============================================================
void MainWindow::autoSuggestLEDs() {
    if (m_origin.isNull()) return;
    int targetCount = s_autoSense ? s_autoSense->value() : 0;
    if (targetCount <= 0) {
        m_ledStrips.clear();
        scheduleProcess();
        return;
    }
    m_ledStrips = m_ledLayoutEngine.autoSuggestLEDs(m_origin, targetCount, s_ledRad ? s_ledRad->value() : 150);
    m_statusLabel->setText(QString("自动布灯完成，共放置 %1 条灯条").arg(m_ledStrips.size()));
    scheduleProcess();
}

void MainWindow::clearAllLEDs() {
    m_ledStrips.clear();
    m_isPlacing = false;
    m_statusLabel->setText("已清除所有灯条");
    scheduleProcess();
}

// ============================================================
//  加载图片
// ============================================================
void MainWindow::loadAndProcess() {
    QString f = QFileDialog::getOpenFileName(this, "选择图片", "",
        "图片文件 (*.png *.jpg *.jpeg *.bmp *.gif *.webp);;所有文件 (*.*)");
    if (f.isEmpty()) return;
    loadImageFromPath(f, false);
}

bool MainWindow::loadImageFromPath(const QString& filePath, bool alreadyInTemp) {
    if (filePath.isEmpty()) return false;

    QString actualExt  = detectImageExtensionFromContent(filePath);
    QString providedExt= QFileInfo(filePath).suffix().toLower();
    if (!actualExt.isEmpty() && !providedExt.isEmpty() && actualExt != providedExt) {
        QMessageBox::warning(this, "格式提示",
            QString("文件实际格式为 .%1，扩展名为 .%2，建议修正。").arg(actualExt).arg(providedExt));
    }

    initTempWorkspace();
    QString sourcePath = filePath;

    if (!alreadyInTemp) {
        QString ext = actualExt.isEmpty() ? providedExt : actualExt;
        if (ext.isEmpty()) ext = "png";
        const QString target = QDir(m_tempDirPath).filePath(QString("source.%1").arg(ext));
        cleanupTempImages(target);
        if (QFile::exists(target)) QFile::remove(target);
        if (!QFile::copy(filePath, target)) {
            QMessageBox::warning(this, "加载失败", "复制图片到临时目录失败，请检查磁盘空间。");
            return false;
        }
        sourcePath = target;
    } else {
        cleanupTempImages(sourcePath);
    }

    QImageReader reader(sourcePath);
    reader.setAutoTransform(true);
    if (!reader.canRead()) {
        if (!actualExt.isEmpty()) {
            QByteArray fmt = actualExt.toLatin1();
            if (fmt == "jpg") fmt = "jpeg";
            reader.setFormat(fmt);
        }
        if (!reader.canRead()) {
            QMessageBox::warning(this, "加载失败", QString("无法读取图片：%1").arg(reader.errorString()));
            return false;
        }
    }

    const QSize srcSize = reader.size();
    if (srcSize.isValid() && !srcSize.isEmpty()) {
        const qint64 maxPx = getMaxImportPixels();
        const qint64 srcPx = (qint64)srcSize.width() * srcSize.height();
        if (srcPx >= maxPx) {
            QSize ts = scaleDownToPixelLimit(srcSize, maxPx);
            if (ts.isValid() && ts != srcSize) {
                reader.setScaledSize(ts);
                m_statusLabel->setText(QString("图片已缩放导入 %1×%2 → %3×%4")
                    .arg(srcSize.width()).arg(srcSize.height()).arg(ts.width()).arg(ts.height()));
            }
        }
    }

    QImage loaded = reader.read();
    if (loaded.isNull()) {
        QMessageBox::warning(this, "加载失败", QString("读取图片失败：%1").arg(reader.errorString()));
        return false;
    }

    m_origin = loaded.convertToFormat(QImage::Format_RGB32);
    m_tempImagePath = sourcePath;
    m_previewZoom = 1.0;
    m_previewPan  = QPointF(0,0);
    m_isPanningPreview = false;
    m_isPlacing = false;
    m_ledStrips.clear();

    if (btn_export) btn_export->setEnabled(true);
    if (action_exportLayers) action_exportLayers->setEnabled(true);

    m_tempImageMTimeMs = fileStampMs(QFileInfo(m_tempImagePath));
    m_tempImageSize    = QFileInfo(m_tempImagePath).size();

    m_statusLabel->setText(QString("已加载  %1  (%2 × %3 px)")
        .arg(QFileInfo(filePath).fileName())
        .arg(m_origin.width()).arg(m_origin.height()));

    scheduleProcess();
    syncArgsToJson();
    return true;
}

// ============================================================
//  导出
// ============================================================
void MainWindow::exportLayers() {
    if (m_layers.isEmpty()) {
        QMessageBox::warning(this, "提示", "请先导入并处理图片后再导出。");
        return;
    }
    QString d = QFileDialog::getExistingDirectory(this, "选择导出目录");
    if (d.isEmpty()) return;

    const QString finishName = (combo_surfaceFinish && combo_surfaceFinish->currentText().contains("沉金")) ? "ENIG" : "HASL";
    if (m_layerGenerator.exportLayersToFiles(m_layers, d, finishName, m_ledStrips)) {
        QMessageBox::information(this, "导出成功",
            QString("生产层已导出到：\n%1\n\n共 %2 个文件").arg(d).arg(m_layers.size() + 1));
    } else {
        QMessageBox::warning(this, "导出失败", "写入文件失败，请检查目标目录权限。");
    }
}

// ============================================================
//  工程保存 / 导入
// ============================================================
void MainWindow::saveProject() {
    if (m_tempImagePath.isEmpty() || !QFile::exists(m_tempImagePath)) {
        QMessageBox::warning(this, "保存失败", "请先导入图片，再保存工程。");
        return;
    }
    QString path = QFileDialog::getSaveFileName(this, "保存工程", "",
        "PCB Lightgraph Project (*.pcblg)");
    if (path.isEmpty()) return;
    if (!path.endsWith(".pcblg", Qt::CaseInsensitive)) path += ".pcblg";
    syncArgsToJson();
    if (!saveProjectToBlg(path)) {
        QMessageBox::warning(this, "保存失败", "工程打包失败，请检查路径或权限。");
        return;
    }
    QMessageBox::information(this, "保存成功", "工程已保存。");
}

void MainWindow::importProject() {
    QString path = QFileDialog::getOpenFileName(this, "导入工程", "",
        "PCB Lightgraph Project (*.pcblg)");
    if (path.isEmpty()) return;
    if (!importProjectFromBlg(path)) {
        QMessageBox::warning(this, "导入失败", "无法导入工程，文件可能损坏。");
        return;
    }
    QMessageBox::information(this, "导入成功", "工程已加载。");
}

bool MainWindow::saveProjectToBlg(const QString& blgPath) {
    initTempWorkspace();
    syncArgsToJson();
    QString srcImg = m_tempImagePath;
    if (srcImg.isEmpty() || !QFile::exists(srcImg)) {
        QDirIterator it(m_tempDirPath, QDir::Files);
        while (it.hasNext()) {
            const QString p = it.next();
            if (isSupportedImageExtension(QFileInfo(p).suffix().toLower())) { srcImg = p; break; }
        }
    }
    if (srcImg.isEmpty() || !QFile::exists(srcImg) || !QFile::exists(m_tempArgsPath)) return false;

    QTemporaryDir stageDir;
    if (!stageDir.isValid()) return false;
    const QString si = QDir(stageDir.path()).filePath(QFileInfo(srcImg).fileName());
    const QString sa = QDir(stageDir.path()).filePath("args.json");
    if (!QFile::copy(srcImg, si) || !QFile::copy(m_tempArgsPath, sa)) return false;

    const QString zipTmp = QDir(stageDir.path()).filePath("project.zip");
    const QString cmd = QString(
        "$ErrorActionPreference='Stop'; Compress-Archive -LiteralPath @(%1,%2) -DestinationPath %3 -Force")
        .arg(psSingleQuoted(si), psSingleQuoted(sa), psSingleQuoted(zipTmp));
    QProcess proc;
    proc.start("powershell", {"-NoProfile", "-Command", cmd});
    if (!proc.waitForFinished(60000) || proc.exitCode() != 0) return false;
    if (QFile::exists(blgPath)) QFile::remove(blgPath);
    return QFile::copy(zipTmp, blgPath);
}

bool MainWindow::importProjectFromBlg(const QString& blgPath) {
    if (blgPath.isEmpty() || !QFile::exists(blgPath)) return false;
    initTempWorkspace();
    QTemporaryDir unpack;
    if (!unpack.isValid()) return false;
    const QString zipPath = QDir(unpack.path()).filePath("project.zip");
    if (!QFile::copy(blgPath, zipPath)) return false;
    const QString extractRoot = QDir(unpack.path()).filePath("content");
    QDir().mkpath(extractRoot);
    const QString cmd = QString(
        "$ErrorActionPreference='Stop'; Expand-Archive -LiteralPath %1 -DestinationPath %2 -Force")
        .arg(psSingleQuoted(zipPath), psSingleQuoted(extractRoot));
    QProcess proc;
    proc.start("powershell", {"-NoProfile", "-Command", cmd});
    if (!proc.waitForFinished(60000) || proc.exitCode() != 0) return false;

    QString imgPath, argsPath;
    QDirIterator it(extractRoot, QDir::Files, QDirIterator::Subdirectories);
    while (it.hasNext()) {
        const QString p = it.next();
        const QFileInfo fi(p);
        if (fi.fileName().toLower() == "args.json" && argsPath.isEmpty()) { argsPath = p; continue; }
        if (isSupportedImageExtension(fi.suffix().toLower()) && imgPath.isEmpty()) imgPath = p;
    }
    if (imgPath.isEmpty() || argsPath.isEmpty()) return false;
    if (!loadImageFromPath(imgPath, false)) return false;
    if (QFile::exists(m_tempArgsPath)) QFile::remove(m_tempArgsPath);
    if (!QFile::copy(argsPath, m_tempArgsPath)) return false;
    const bool ok = loadArgsFromJson(m_tempArgsPath);
    if (ok) {
        m_tempImageMTimeMs = fileStampMs(QFileInfo(m_tempImagePath));
        m_tempImageSize    = QFileInfo(m_tempImagePath).size();
    }
    return ok;
}

// ============================================================
//  JSON 参数同步
// ============================================================
void MainWindow::syncArgsToJson() {
    initTempWorkspace();
    QJsonObject root;
    root["schemaVersion"] = 2;

    QJsonObject c;
    c["surfaceFinishIndex"] = combo_surfaceFinish ? combo_surfaceFinish->currentIndex() : 0;
    c["maskColorIndex"]     = combo_maskColor     ? combo_maskColor->currentIndex()     : 0;
    c["goldThresh"]    = s_gold        ? s_gold->value()        : 45;
    c["silkThresh"]    = s_silk        ? s_silk->value()        : 180;
    c["transThresh"]   = s_trans       ? s_trans->value()       : 120;
    c["copperDepth"]   = s_copperDepth ? s_copperDepth->value() : 150;
    c["lightEnable"]   = check_lightEnable    && check_lightEnable->isChecked();
    c["showLEDOverlay"]= check_showLEDOverlay && check_showLEDOverlay->isChecked();
    c["autoSense"]     = s_autoSense   ? s_autoSense->value()   : 1;
    c["ledRadius"]     = s_ledRad      ? s_ledRad->value()      : 150;
    c["ledIntensity"]  = s_ledIntensity? s_ledIntensity->value(): 200;
    c["bareSubstrateEnable"]   = (m_grpBare && m_grpBare->isChecked());
    c["bareSubstrateGrayMode"] = radio_bareSubstrateGray && radio_bareSubstrateGray->isChecked();
    c["bareSubstrateGrayA"]    = s_bareSubstrateGrayA     ? s_bareSubstrateGrayA->value()     : 20;
    c["bareSubstrateGrayB"]    = s_bareSubstrateGrayB     ? s_bareSubstrateGrayB->value()     : 65;
    c["bareSubstrateColorSimilarity"] = s_bareSubstrateColorSimilarity ? s_bareSubstrateColorSimilarity->value() : 80;
    c["edgeEnable"]    = group_edgeOperation && group_edgeOperation->isChecked();
    c["edgeMode"]      = (radio_edgeStroke && radio_edgeStroke->isChecked()) ? "stroke" : "enhance";
    c["edgeThreshMin"] = s_edgeThresh    ? s_edgeThresh->value()    : 50;
    c["edgeThreshMax"] = s_edgeThreshMax ? s_edgeThreshMax->value() : 200;
    c["autoInvert"]    = s_autoInvert    ? s_autoInvert->value()    : 10;
    c["useMetalEdge"]  = check_useMetalEdge    && check_useMetalEdge->isChecked();
    c["exposeMetalEdge"]= check_exposeMetalEdge && check_exposeMetalEdge->isChecked();
    root["controls"] = c;

    QJsonObject exp;
    exp["edgePrefilterEnabled"]    = m_edgePrefilterEnabled;
    exp["edgePrefilterKernelSize"] = m_edgePrefilterKernelSize;
    exp["edgePrefilterSigma"]      = m_edgePrefilterSigma;
    exp["photoPreprocessEnabled"]  = m_photoPreprocessEnabled;
    exp["denoiseEnabled"]          = m_denoiseEnabled;
    exp["posterizeEnabled"]        = m_posterizeEnabled;
    exp["photoPreprocessKernelSize"]= m_photoPreprocessKernelSize;
    exp["photoPosterizeLevels"]    = m_photoPosterizeLevels;
    exp["layerCleanupEnabled"]     = m_layerCleanupEnabled;
    exp["layerCleanupMinArea"]     = m_layerCleanupMinArea;
    exp["dpEnabled"]    = m_dpEnabled;
    exp["dpTolerance"]  = m_dpTolerance;
    exp["dpLineWidth"]  = m_dpLineWidth;
    root["experimental"] = exp;

    QJsonArray strips;
    for (const LEDStrip& s : m_ledStrips) {
        QJsonObject o;
        o["startX"]=s.start.x(); o["startY"]=s.start.y();
        o["endX"]  =s.end.x();   o["endY"]  =s.end.y();
        o["radius"]=s.radius;
        o["r"]=s.color.red(); o["g"]=s.color.green();
        o["b"]=s.color.blue(); o["a"]=s.color.alpha();
        strips.append(o);
    }
    root["ledStrips"] = strips;
    if (!m_tempImagePath.isEmpty())
        root["imageFileName"] = QFileInfo(m_tempImagePath).fileName();

    QSaveFile sf(m_tempArgsPath);
    if (!sf.open(QIODevice::WriteOnly | QIODevice::Truncate)) return;
    sf.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    sf.commit();
}

bool MainWindow::loadArgsFromJson(const QString& argsPath) {
    QFile f(argsPath);
    if (!f.open(QIODevice::ReadOnly)) return false;
    QJsonParseError err;
    const QJsonDocument doc = QJsonDocument::fromJson(f.readAll(), &err);
    f.close();
    if (err.error != QJsonParseError::NoError || !doc.isObject()) return false;

    const QJsonObject root = doc.object();
    const QJsonObject c    = root.value("controls").toObject();
    const QJsonObject exp  = root.value("experimental").toObject();

    auto setSlider = [](QSlider* s, int v) {
        if (!s) return;
        QSignalBlocker b(s);
        s->setValue(qBound(s->minimum(), v, s->maximum()));
    };
    auto setCheck = [](QCheckBox* cb, bool v) { if (cb) cb->setChecked(v); };
    auto setRadio = [](QRadioButton* r, bool v) {
        if (!r) return;
        QSignalBlocker b(r);
        r->setChecked(v);
    };
    auto setCombo = [](QComboBox* cb, int i) {
        if (!cb) return;
        QSignalBlocker b(cb);
        cb->setCurrentIndex(qBound(0, i, cb->count()-1));
    };

    m_isApplyingArgs = true;

    setCombo(combo_surfaceFinish, c.value("surfaceFinishIndex").toInt(0));
    setCombo(combo_maskColor,     c.value("maskColorIndex").toInt(0));
    setSlider(s_gold,        c.value("goldThresh").toInt(45));
    setSlider(s_silk,        c.value("silkThresh").toInt(180));
    setSlider(s_trans,       c.value("transThresh").toInt(120));
    setSlider(s_copperDepth, c.value("copperDepth").toInt(150));
    setCheck(check_lightEnable,    c.value("lightEnable").toBool(false));
    setCheck(check_showLEDOverlay, c.value("showLEDOverlay").toBool(false));
    setSlider(s_autoSense,    c.value("autoSense").toInt(1));
    setSlider(s_ledRad,       c.value("ledRadius").toInt(150));
    setSlider(s_ledIntensity, c.value("ledIntensity").toInt(200));

    bool bareOn = c.value("bareSubstrateEnable").toBool(false);
    setCheck(check_bareSubstrateEnable, bareOn);
    bool grayMode = c.value("bareSubstrateGrayMode").toBool(true);
    setRadio(radio_bareSubstrateGray,  grayMode);
    setRadio(radio_bareSubstrateColor, !grayMode);
    setSlider(s_bareSubstrateGrayA,     c.value("bareSubstrateGrayA").toInt(20));
    setSlider(s_bareSubstrateGrayB,     c.value("bareSubstrateGrayB").toInt(65));
    setSlider(s_bareSubstrateColorSimilarity, c.value("bareSubstrateColorSimilarity").toInt(80));

    bool edgeOn = c.value("edgeEnable").toBool(false);
    if (group_edgeOperation) {
        QSignalBlocker b(group_edgeOperation);
        group_edgeOperation->setChecked(edgeOn);
    }
    bool isStroke = (c.value("edgeMode").toString("enhance") == "stroke");
    setRadio(radio_edgeStroke,  isStroke);
    setRadio(radio_edgeEnhance, !isStroke);
    setSlider(s_edgeThresh,    c.value("edgeThreshMin").toInt(50));
    setSlider(s_edgeThreshMax, c.value("edgeThreshMax").toInt(200));
    setSlider(s_autoInvert,    c.value("autoInvert").toInt(10));
    setCheck(check_useMetalEdge,    c.value("useMetalEdge").toBool(false));
    setCheck(check_exposeMetalEdge, c.value("exposeMetalEdge").toBool(true));

    m_edgePrefilterEnabled    = exp.value("edgePrefilterEnabled").toBool(true);
    m_edgePrefilterKernelSize = exp.value("edgePrefilterKernelSize").toInt(5);
    m_edgePrefilterSigma      = exp.value("edgePrefilterSigma").toDouble(1.1);
    m_denoiseEnabled          = exp.value("denoiseEnabled").toBool(true);
    m_posterizeEnabled        = exp.value("posterizeEnabled").toBool(true);
    m_photoPreprocessEnabled  = m_denoiseEnabled || m_posterizeEnabled;
    m_photoPreprocessKernelSize= clampOddKernel(exp.value("photoPreprocessKernelSize").toInt(7), 3, 11);
    m_photoPosterizeLevels    = qBound(2, exp.value("photoPosterizeLevels").toInt(16), 64);
    m_layerCleanupEnabled     = exp.value("layerCleanupEnabled").toBool(false);
    m_layerCleanupMinArea     = qBound(2, exp.value("layerCleanupMinArea").toInt(24), 5000);
    m_dpEnabled               = exp.value("dpEnabled").toBool(false);
    m_dpTolerance             = exp.value("dpTolerance").toDouble(1.0);
    m_dpLineWidth             = exp.value("dpLineWidth").toInt(2);

    m_ledStrips.clear();
    for (const QJsonValue& v : root.value("ledStrips").toArray()) {
        if (!v.isObject()) continue;
        const QJsonObject o = v.toObject();
        LEDStrip s;
        s.start  = QPoint(o.value("startX").toInt(), o.value("startY").toInt());
        s.end    = QPoint(o.value("endX").toInt(),   o.value("endY").toInt());
        s.radius = o.value("radius").toInt(150);
        s.color  = QColor(o.value("r").toInt(255), o.value("g").toInt(255),
                          o.value("b").toInt(255), o.value("a").toInt(255));
        m_ledStrips.append(s);
    }

    m_isApplyingArgs = false;
    scheduleProcess();
    return true;
}

// ============================================================
//  临时工作区
// ============================================================
void MainWindow::initTempWorkspace() {
    if (m_tempDirPath.isEmpty())
        m_tempDirPath = QDir(QCoreApplication::applicationDirPath()).filePath("temp");
    QDir d(m_tempDirPath);
    if (!d.exists()) d.mkpath(".");
    m_tempArgsPath = d.filePath("args.json");
}

void MainWindow::cleanupTempImages(const QString& keepImagePath) {
    initTempWorkspace();
    const QString keepAbs = keepImagePath.isEmpty() ? QString() : QFileInfo(keepImagePath).absoluteFilePath();
    for (const QFileInfo& fi : QDir(m_tempDirPath).entryInfoList(QDir::Files | QDir::NoDotAndDotDot)) {
        if (!isSupportedImageExtension(fi.suffix().toLower())) continue;
        const QString abs = fi.absoluteFilePath();
        if (!keepAbs.isEmpty() && abs.compare(keepAbs, Qt::CaseInsensitive)==0) continue;
        QFile::remove(abs);
        if (abs.compare(m_tempImagePath, Qt::CaseInsensitive)==0) m_tempImagePath.clear();
    }
}

QString MainWindow::resolveCurrentTempImagePath() const {
    if (m_tempDirPath.isEmpty()) return {};
    const QString name = QFileInfo(m_tempImagePath).fileName();
    if (!name.isEmpty()) {
        const QString p = QDir(m_tempDirPath).filePath(name);
        if (QFile::exists(p)) return p;
    }
    for (const QFileInfo& fi : QDir(m_tempDirPath).entryInfoList(QDir::Files, QDir::Time | QDir::Reversed))
        if (isSupportedImageExtension(fi.suffix().toLower())) return fi.absoluteFilePath();
    return {};
}

void MainWindow::checkTempImageUpdated() {
    if (m_tempImagePath.isEmpty() || !QFile::exists(m_tempImagePath)) return;
    const QFileInfo fi(m_tempImagePath);
    const qint64 stamp = fileStampMs(fi);
    const qint64 size  = fi.size();
    if (m_tempImageMTimeMs < 0) { m_tempImageMTimeMs = stamp; m_tempImageSize = size; return; }
    if (stamp == m_tempImageMTimeMs && size == m_tempImageSize) return;
    m_tempImageMTimeMs = stamp;
    m_tempImageSize    = size;
    if (m_isApplyingArgs) return;
    if (!loadImageFromPath(m_tempImagePath, true))
        m_statusLabel->setText("检测到图片变更，但重载失败。");
}

// ============================================================
//  画图实时编辑
// ============================================================
void MainWindow::openPaintEditor() {
    initTempWorkspace();
    QString tempImg = resolveCurrentTempImagePath();
    if (tempImg.isEmpty() || !QFile::exists(tempImg)) {
        QMessageBox::warning(this, "提示",
            QString("未能在临时目录找到当前图片。\n临时目录：%1").arg(m_tempDirPath));
        return;
    }
    QMessageBox::information(this, "提示",
        "将打开画图程序。\n在画图中按 Ctrl+S 保存后，本程序会自动重新处理。");
    QProcess::startDetached("mspaint",
        {QDir::toNativeSeparators(QFileInfo(tempImg).absoluteFilePath())});
}

// ============================================================
//  实验性功能对话框
// ============================================================
void MainWindow::openPhotoPreprocessDialog() {
    QDialog dlg(this);
    dlg.setWindowTitle("照片去噪 / 生产层清理");
    QVBoxLayout* lay = new QVBoxLayout(&dlg);

    QCheckBox* preCheck = new QCheckBox("启用分层前去噪和颜色量化");
    preCheck->setChecked(m_photoPreprocessEnabled);
    lay->addWidget(preCheck);

    QHBoxLayout* kl = new QHBoxLayout;
    kl->addWidget(new QLabel("去噪核大小:"));
    QSpinBox* kSpin = new QSpinBox; kSpin->setRange(3,11); kSpin->setSingleStep(2);
    kSpin->setValue(clampOddKernel(m_photoPreprocessKernelSize,3,11));
    kl->addWidget(kSpin); lay->addLayout(kl);

    QHBoxLayout* pl = new QHBoxLayout;
    pl->addWidget(new QLabel("颜色量化级数:"));
    QSpinBox* pSpin = new QSpinBox; pSpin->setRange(2,64);
    pSpin->setValue(qBound(2,m_photoPosterizeLevels,64));
    pl->addWidget(pSpin); lay->addLayout(pl);

    QCheckBox* cleanCheck = new QCheckBox("启用生产层小碎片/小孔清理");
    cleanCheck->setChecked(m_layerCleanupEnabled);
    lay->addWidget(cleanCheck);

    QHBoxLayout* al = new QHBoxLayout;
    al->addWidget(new QLabel("最小连通域面积 (像素):"));
    QSpinBox* aSpin = new QSpinBox; aSpin->setRange(2,5000);
    aSpin->setValue(qBound(2,m_layerCleanupMinArea,5000));
    al->addWidget(aSpin); lay->addLayout(al);

    auto updateState = [=]() {
        kSpin->setEnabled(preCheck->isChecked());
        pSpin->setEnabled(preCheck->isChecked());
        aSpin->setEnabled(cleanCheck->isChecked());
    };
    connect(preCheck,  &QCheckBox::toggled, &dlg, updateState);
    connect(cleanCheck,&QCheckBox::toggled, &dlg, updateState);
    updateState();

    QDialogButtonBox* btns = new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
    lay->addWidget(btns);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        m_photoPreprocessEnabled  = preCheck->isChecked();
        m_photoPreprocessKernelSize= clampOddKernel(kSpin->value(),3,11);
        m_photoPosterizeLevels    = qBound(2,pSpin->value(),64);
        m_layerCleanupEnabled     = cleanCheck->isChecked();
        m_layerCleanupMinArea     = qBound(2,aSpin->value(),5000);
        if (m_statusLabel) m_statusLabel->setText("去噪参数已应用，重新处理中...");
        scheduleProcess();
    }
}

void MainWindow::openFilterPreprocessDialog() {
    QDialog dlg(this);
    dlg.setWindowTitle("边缘滤波预处理");
    QVBoxLayout* lay = new QVBoxLayout(&dlg);

    QCheckBox* enCheck = new QCheckBox("开启高斯预滤波（减少噪点，轻微模糊细节）");
    enCheck->setChecked(m_edgePrefilterEnabled);
    lay->addWidget(enCheck);

    QLabel* kLabel = new QLabel;
    QSlider* kSlider = new QSlider(Qt::Horizontal);
    kSlider->setRange(3,7); kSlider->setSingleStep(2); kSlider->setPageStep(2);
    kSlider->setValue(m_edgePrefilterKernelSize);
    lay->addWidget(kLabel); lay->addWidget(kSlider);

    auto updateKLabel = [kLabel](int v){ kLabel->setText(QString("高斯核大小: %1").arg(v)); };
    auto snapOdd = [kSlider, updateKLabel](int v) {
        int s = (v%2==0) ? (v>=5?v+1:v-1) : v;
        s = qBound(3, s, 7);
        if (s != v) { QSignalBlocker b(kSlider); kSlider->setValue(s); }
        updateKLabel(s);
    };
    connect(kSlider, &QSlider::valueChanged, &dlg, snapOdd);
    connect(enCheck, &QCheckBox::toggled, kSlider, &QWidget::setEnabled);
    kSlider->setEnabled(enCheck->isChecked());
    updateKLabel(kSlider->value());

    QDialogButtonBox* btns = new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
    lay->addWidget(btns);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        m_edgePrefilterEnabled    = enCheck->isChecked();
        m_edgePrefilterKernelSize = kSlider->value();
        scheduleProcess();
    }
}

void MainWindow::openDouglasPeuckerDialog() {
    QDialog dlg(this);
    dlg.setWindowTitle("道格拉斯-普克 抽稀设置");
    QVBoxLayout* lay = new QVBoxLayout(&dlg);

    QCheckBox* enCheck = new QCheckBox("启用道格拉斯-普克抽稀");
    enCheck->setChecked(m_dpEnabled);
    lay->addWidget(enCheck);

    QHBoxLayout* tl = new QHBoxLayout;
    tl->addWidget(new QLabel("抽稀容忍度 (epsilon):"));
    QDoubleSpinBox* tSpin = new QDoubleSpinBox;
    tSpin->setRange(0.0, 1000.0); tSpin->setDecimals(3); tSpin->setSingleStep(0.1);
    tSpin->setValue(m_dpTolerance);
    tl->addWidget(tSpin); lay->addLayout(tl);

    QHBoxLayout* wl = new QHBoxLayout;
    wl->addWidget(new QLabel("重绘线宽 (像素):"));
    QSpinBox* wSpin = new QSpinBox; wSpin->setRange(1,50); wSpin->setValue(m_dpLineWidth);
    wl->addWidget(wSpin); lay->addLayout(wl);

    QDialogButtonBox* btns = new QDialogButtonBox(QDialogButtonBox::Ok|QDialogButtonBox::Cancel);
    lay->addWidget(btns);
    connect(btns, &QDialogButtonBox::accepted, &dlg, &QDialog::accept);
    connect(btns, &QDialogButtonBox::rejected, &dlg, &QDialog::reject);

    if (dlg.exec() == QDialog::Accepted) {
        m_dpEnabled   = enCheck->isChecked();
        m_dpTolerance = qMax(0.0, tSpin->value());
        m_dpLineWidth = qMax(1,   wSpin->value());
        scheduleProcess();
    }
}
