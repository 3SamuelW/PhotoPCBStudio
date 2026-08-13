#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QImage>
#include <QLabel>
#include <QSlider>
#include <QPushButton>
#include <QMap>
#include <QVector>
#include <QPoint>
#include <QComboBox>
#include <QPainter>
#include <QCheckBox>
#include <QRadioButton>
#include <QGroupBox>
#include <QProgressBar>
#include <QFutureWatcher>
#include <QTimer>
#include <QMutex>
#include <QVBoxLayout>

class QAction;
class QScrollArea;
class QStackedWidget;

#include "imageprocessor.h"
#include "edgesharpener.h"
#include "ledlayoutengine.h"
#include "layergenerator.h"

// 图像处理结果，跨线程传递
struct ProcessResult {
    QImage copper, mask, silk, bottom, composite;
    QMap<QString, QImage> layers;
    bool valid = false;
};

class MainWindow : public QMainWindow {
    Q_OBJECT

    struct PreviewState {
        double zoom = 1.0;
        QPointF pan = QPointF(0, 0);
        bool isPanning = false;
        QPoint lastPanPos;
    };

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();
    bool eventFilter(QObject *obj, QEvent *event) override;

protected:
    void resizeEvent(QResizeEvent *event) override;

private slots:
    void scheduleProcess();       // 防抖：重置计时器
    void triggerProcess();         // 防抖到期：真正触发
    void onProcessFinished();      // 后台线程完成
    void loadAndProcess();
    void exportLayers();
    void saveProject();
    void importProject();
    void openPaintEditor();
    void autoSuggestLEDs();
    void openFilterPreprocessDialog();
    void openPhotoPreprocessDialog();
    void openDouglasPeuckerDialog();
    void clearAllLEDs();

private:
    void setupUI();
    void setupMenuBar();
    QSlider* createSlider(const QString& title, int min, int max, int def, QVBoxLayout* layout);
    QGroupBox* createCollapsibleGroup(const QString& title, QWidget* content, bool collapsed = true);

    void updateCompositePreview(const QImage& img);
    void updateLayerPreview(QLabel* label, const QImage& img, PreviewState& state);
    void clampPreviewPan();
    void clampPreviewPan(QLabel* label, const QImage& img, PreviewState& state);
    bool handleLayerPreviewEvent(QLabel* label, QEvent* event, const QImage& img, PreviewState& state);
    bool mapLabelToImage(const QPoint& labelPos, QPoint& imgPos) const;

    void initTempWorkspace();
    void cleanupTempImages(const QString& keepImagePath = QString());
    bool loadImageFromPath(const QString& filePath, bool alreadyInTemp = false);
    QString resolveCurrentTempImagePath() const;
    void syncArgsToJson();
    bool loadArgsFromJson(const QString& argsPath);
    bool saveProjectToBlg(const QString& blgPath);
    bool importProjectFromBlg(const QString& blgPath);
    void checkTempImageUpdated();

    void setProcessingState(bool processing);
    void applyLayerResults(const ProcessResult& res);

    // 收集当前所有 UI 参数，线程安全地传给后台
    struct ProcessParams {
        QImage srcImage;
        int goldThresh, silkThresh, transThresh, copperDepth;
        int ledRadVal, ledIntensity;
        QString maskColorName, finishType;
        bool isWhiteMask;
        bool enableBareSubstrate, bareSubstrateUseGrayBinding;
        int bareSubstrateGrayMinPct, bareSubstrateGrayMaxPct, bareSubstrateColorSimilarityPct;
        bool edgeEnabled;
        EdgeSharpener::OperationMode edgeMode;
        int edgeThreshMin, edgeThreshMax, autoInvert;
        bool useMetalEdge, exposeMetalEdge;
        bool edgePrefilterEnabled;
        int edgePrefilterKernelSize;
        double edgePrefilterSigma;
        bool photoPreprocessEnabled;
        bool denoiseEnabled;       // 去噪独立开关
        bool posterizeEnabled;     // 量化独立开关
        int photoPreprocessKernelSize, photoPosterizeLevels;
        bool layerCleanupEnabled;
        int layerCleanupMinArea;
        bool dpEnabled;
        double dpTolerance;
        int dpLineWidth;
        bool showOverlay;
        QVector<LEDStrip> ledStrips;
    };
    ProcessParams collectParams() const;
    static ProcessResult runProcessing(ProcessParams p);

    // ----------- 图像数据 -----------
    QImage m_origin;
    QImage processedOrigin;
    QImage m_previewComposite;
    QMap<QString, QImage> m_layers;
    QVector<LEDStrip> m_ledStrips;

    // ----------- 预览状态 -----------
    QPoint m_pendingStart;
    bool m_isPlacing = false;
    bool m_isPanningPreview = false;
    QPoint m_lastPanPos;
    QPointF m_previewPan;
    double m_previewZoom = 1.0;
    QMap<QLabel*, QString> m_layerPreviewKeys;
    QMap<QLabel*, PreviewState> m_layerPreviewStates;
    QMap<QLabel*, QString> m_layerPreviewTitles;  // for overlay text

    // ----------- UI 组件 -----------
    QLabel *l_copper, *l_mask, *l_silk, *l_bottom, *l_composite;
    QSlider *s_silk, *s_gold, *s_trans, *s_ledRad;
    QComboBox *combo_maskColor;
    QPushButton *btn_export;
    QAction *action_exportLayers = nullptr;
    QComboBox *combo_surfaceFinish;
    QCheckBox *check_lightEnable;
    QCheckBox *check_expandPreviews;
    QGroupBox *group_edgeOperation;
    QCheckBox *check_edgeEnable;
    QRadioButton *radio_edgeStroke;
    QRadioButton *radio_edgeEnhance;
    QSlider *s_edgeThresh, *s_autoInvert, *s_edgeThreshMax;
    QSlider *s_autoSense;
    QSlider *s_copperDepth;
    QSlider *s_ledIntensity;
    QCheckBox *check_showLEDOverlay;
    QCheckBox *check_bareSubstrateEnable;
    QRadioButton *radio_bareSubstrateGray;
    QRadioButton *radio_bareSubstrateColor;
    QSlider *s_bareSubstrateGrayA;
    QSlider *s_bareSubstrateGrayB;
    QSlider *s_bareSubstrateColorSimilarity;
    QCheckBox *check_useMetalEdge;
    QCheckBox *check_exposeMetalEdge;
    QProgressBar *m_progressBar;
    QLabel *m_statusLabel;
    QWidget *m_rightPanel = nullptr;
    QGroupBox *m_grpBare = nullptr;   // 裸露基材折叠组

    // ----------- 异步处理 -----------
    QFutureWatcher<ProcessResult> *m_watcher = nullptr;
    QTimer *m_debounceTimer = nullptr;
    bool m_processingInFlight = false;
    bool m_pendingRequest = false;  // 处理进行中时又有新请求

    // ----------- 实验性参数 -----------
    bool m_edgePrefilterEnabled = true;
    int m_edgePrefilterKernelSize = 5;
    double m_edgePrefilterSigma = 1.1;
    bool m_photoPreprocessEnabled = true;   // 总开关（去噪 OR 量化 任一开则为true）
    bool m_denoiseEnabled = true;           // 去噪独立开关
    bool m_posterizeEnabled = true;         // 量化独立开关
    int m_photoPreprocessKernelSize = 7;    // 中等去噪
    int m_photoPosterizeLevels = 16;        // 适度量化
    bool m_layerCleanupEnabled = false;
    int m_layerCleanupMinArea = 24;
    bool m_dpEnabled = false;
    double m_dpTolerance = 1.0;
    int m_dpLineWidth = 2;

    // ----------- 工程临时工作区 -----------
    QString m_tempDirPath;
    QString m_tempImagePath;
    QString m_tempArgsPath;
    bool m_isApplyingArgs = false;
    QTimer *m_tempReloadTimer = nullptr;
    qint64 m_tempImageMTimeMs = -1;
    qint64 m_tempImageSize = -1;

    // ----------- 子模块实例 -----------
    LayerGenerator m_layerGenerator;
    LEDLayoutEngine m_ledLayoutEngine;
};

#endif // MAINWINDOW_H
