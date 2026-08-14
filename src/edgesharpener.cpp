// edgesharpener.cpp — all pixel access via scanLine (no pixel()/setPixel calls)
#include "edgesharpener.h"
#include "gaussian_blur.h"
#include "dp_simplify.h"
#include <cmath>
#include <QDebug>
#include <QVector>
#include <algorithm>
#include <numeric>
#include <QImage>
#include <QPainter>
#include <QPen>
#include <QPoint>

EdgeSharpener::EdgeSharpener() {}
EdgeSharpener::~EdgeSharpener() {}

// Simple content hash for cache key — samples ~1000 pixels + mixes in size
static quint32 imageContentHash(const QImage& img) {
    if (img.isNull()) return 0;
    const int w = img.width(), h = img.height();
    quint32 hash = 2166136261u;
    const int step = qMax(1, static_cast<int>(std::sqrt(static_cast<double>(w * h) / 1000.0)));
    for (int y = 0; y < h; y += step) {
        const QRgb* line = reinterpret_cast<const QRgb*>(img.constScanLine(y));
        for (int x = 0; x < w; x += step) {
            hash ^= static_cast<quint32>(line[x]);
            hash *= 16777619u;
        }
    }
    hash ^= static_cast<quint32>(w * 65537 + h);
    hash *= 16777619u;
    return hash;
}

void EdgeSharpener::buildCacheIfNeeded(const QImage& srcImage) {
    if (srcImage.isNull()) return;

    // Use content hash + size as cache key (cacheKey() changes on every copy)
    quint64 key = (static_cast<quint64>(imageContentHash(srcImage)) << 32)
                | (static_cast<quint64>(srcImage.width()) << 16)
                | static_cast<quint64>(srcImage.height());

    if (m_cachedImageKey == key && m_cachedW == srcImage.width() && m_cachedH == srcImage.height())
        return;

    m_cachedImageKey = key;
    m_cachedW = srcImage.width();
    m_cachedH = srcImage.height();

    const int w = m_cachedW;
    const int h = m_cachedH;

    // Ensure RGB32 for consistent scanLine access
    QImage img = srcImage.convertToFormat(QImage::Format_RGB32);

    m_gray.clear();
    m_edgeStrength.clear();
    m_integral.clear();

    m_gray.resize(w * h);
    m_edgeStrength.resize(w * h);
    m_integral.resize((w + 1) * (h + 1));
    m_integral.fill(0ULL);  // zero-init

    m_sobelMagnitude.clear();
    m_sobelDirection.clear();
    m_cannyNms.clear();
    m_cannyCacheReady = false;

    // Fill gray via scanLine (no pixel() calls)
    for (int y = 0; y < h; ++y) {
        const QRgb* line = reinterpret_cast<const QRgb*>(img.constScanLine(y));
        int* gline = &m_gray[y * w];
        for (int x = 0; x < w; ++x)
            gline[x] = qGray(line[x]);
    }

    // Laplacian edge strength (4-connected, border stays 0)
    std::fill(m_edgeStrength.begin(), m_edgeStrength.end(), 0);
    for (int y = 1; y < h - 1; ++y) {
        const int* prev = &m_gray[(y - 1) * w];
        const int* curr = &m_gray[y * w];
        const int* next = &m_gray[(y + 1) * w];
        int* estr = &m_edgeStrength[y * w];
        for (int x = 1; x < w - 1; ++x) {
            int val = std::abs(4 * curr[x] - curr[x-1] - curr[x+1] - prev[x] - next[x]);
            estr[x] = val;
        }
    }

    // Integral image (summed area table)
    const int iw = w + 1;
    std::fill(m_integral.begin(), m_integral.end(), 0ULL);
    for (int j = 1; j <= h; ++j) {
        const int* grow = &m_gray[(j - 1) * w];
        for (int i = 1; i <= w; ++i) {
            m_integral[j * iw + i] =
                m_integral[(j-1) * iw + i]
              + m_integral[j * iw + (i-1)]
              - m_integral[(j-1) * iw + (i-1)]
              + static_cast<quint64>(grow[i-1]);
        }
    }
}

void EdgeSharpener::buildCannyCacheIfNeeded() {
    if (m_cachedW <= 2 || m_cachedH <= 2 || m_cannyCacheReady) return;

    const int w = m_cachedW;
    const int h = m_cachedH;
    m_sobelMagnitude.resize(w * h);
    m_sobelDirection.resize(w * h);
    m_cannyNms.resize(w * h);
    std::fill(m_sobelMagnitude.begin(), m_sobelMagnitude.end(), 0);
    std::fill(m_sobelDirection.begin(), m_sobelDirection.end(), 0);
    std::fill(m_cannyNms.begin(), m_cannyNms.end(), 0);

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;
            const int a = m_gray[(y-1)*w + (x-1)];
            const int b = m_gray[(y-1)*w +  x   ];
            const int c = m_gray[(y-1)*w + (x+1)];
            const int d = m_gray[ y   *w + (x-1)];
            const int f = m_gray[ y   *w + (x+1)];
            const int g = m_gray[(y+1)*w + (x-1)];
            const int hv= m_gray[(y+1)*w +  x   ];
            const int i = m_gray[(y+1)*w + (x+1)];

            const int gx = -a + c - 2*d + 2*f - g + i;
            const int gy =  a + 2*b + c - g - 2*hv - i;
            const int mag = static_cast<int>(std::sqrt(static_cast<double>(gx*gx + gy*gy)));
            m_sobelMagnitude[idx] = mag;

            double angle = std::atan2(static_cast<double>(gy), static_cast<double>(gx)) * 57.29577951308232;
            if (angle < 0.0) angle += 180.0;
            uchar dir = 0;
            if ((angle >= 0.0 && angle < 22.5) || angle >= 157.5) dir = 0;
            else if (angle < 67.5)  dir = 45;
            else if (angle < 112.5) dir = 90;
            else                    dir = 135;
            m_sobelDirection[idx] = dir;
        }
    }

    for (int y = 1; y < h - 1; ++y) {
        for (int x = 1; x < w - 1; ++x) {
            const int idx = y * w + x;
            const int mag = m_sobelMagnitude[idx];
            int m1 = 0, m2 = 0;
            switch (m_sobelDirection[idx]) {
            case 0:   m1 = m_sobelMagnitude[idx-1];            m2 = m_sobelMagnitude[idx+1];            break;
            case 45:  m1 = m_sobelMagnitude[(y-1)*w+(x+1)];   m2 = m_sobelMagnitude[(y+1)*w+(x-1)];   break;
            case 90:  m1 = m_sobelMagnitude[(y-1)*w+x];        m2 = m_sobelMagnitude[(y+1)*w+x];        break;
            default:  m1 = m_sobelMagnitude[(y-1)*w+(x-1)];   m2 = m_sobelMagnitude[(y+1)*w+(x+1)];   break;
            }
            m_cannyNms[idx] = (mag >= m1 && mag >= m2) ? mag : 0;
        }
    }

    m_cannyCacheReady = true;
}

int EdgeSharpener::averageGrayInBox(int x0, int y0, int x1, int y1) const {
    if (m_cachedW <= 0 || m_cachedH <= 0) return 128;
    x0 = qBound(0, x0, m_cachedW - 1);
    x1 = qBound(0, x1, m_cachedW - 1);
    y0 = qBound(0, y0, m_cachedH - 1);
    y1 = qBound(0, y1, m_cachedH - 1);
    if (x1 < x0 || y1 < y0) return 128;
    const int iw = m_cachedW + 1;
    quint64 A = m_integral[ y0      * iw +  x0     ];
    quint64 B = m_integral[ y0      * iw + (x1+1)  ];
    quint64 C = m_integral[(y1+1)   * iw +  x0     ];
    quint64 D = m_integral[(y1+1)   * iw + (x1+1)  ];
    quint64 sum = D + A - B - C;
    quint64 cnt = static_cast<quint64>(x1 - x0 + 1) * static_cast<quint64>(y1 - y0 + 1);
    if (cnt == 0) return 128;
    return static_cast<int>(sum / cnt);
}

// Unused direct method kept for API compatibility
int EdgeSharpener::calculateEdgeStrength(const QImage& img, int x, int y) {
    const int w = img.width(), h = img.height();
    if (x <= 0 || x >= w-1 || y <= 0 || y >= h-1) return 0;
    // Use cached data if available, else fall back to scanLine read
    if (m_cachedW == w && m_cachedH == h)
        return m_edgeStrength[y * w + x];
    QImage safe = img.convertToFormat(QImage::Format_RGB32);
    auto G = [&](int px, int py) {
        return qGray(reinterpret_cast<const QRgb*>(safe.constScanLine(py))[px]);
    };
    return std::abs(4*G(x,y) - G(x-1,y) - G(x+1,y) - G(x,y-1) - G(x,y+1));
}

QImage EdgeSharpener::buildLaplacianMask(int edgeThreshMin, int edgeThreshMax) const {
    const int w = m_cachedW, h = m_cachedH;
    QImage edgeMask(w, h, QImage::Format_Grayscale8);
    edgeMask.fill(0);
    for (int y = 1; y < h-1; ++y) {
        uchar* line = edgeMask.scanLine(y);
        const int* estr = &m_edgeStrength[y * w];
        for (int x = 1; x < w-1; ++x) {
            if (estr[x] > edgeThreshMin && estr[x] < edgeThreshMax)
                line[x] = 255;
        }
    }
    return edgeMask;
}

QImage EdgeSharpener::buildCannyMask(int edgeThreshMin, int edgeThreshMax) {
    buildCannyCacheIfNeeded();
    const int w = m_cachedW, h = m_cachedH;
    int low  = qBound(0, edgeThreshMin, 4096);
    int high = qBound(0, edgeThreshMax, 4096);
    if (low > high) std::swap(low, high);

    QImage edgeMask(w, h, QImage::Format_Grayscale8);
    edgeMask.fill(0);

    QVector<char> visited(w * h, 0);
    QVector<int> stack;
    stack.reserve(4096);

    for (int y = 1; y < h-1; ++y) {
        for (int x = 1; x < w-1; ++x) {
            const int idx = y * w + x;
            if (visited[idx] || m_cannyNms[idx] < high) continue;

            stack.clear();
            stack.append(idx);
            visited[idx] = 1;

            while (!stack.isEmpty()) {
                const int cur = stack.takeLast();
                const int cx = cur % w, cy = cur / w;
                edgeMask.scanLine(cy)[cx] = 255;

                for (int dy = -1; dy <= 1; ++dy) {
                    for (int dx = -1; dx <= 1; ++dx) {
                        if (dx == 0 && dy == 0) continue;
                        const int nx = cx + dx, ny = cy + dy;
                        if (nx <= 0 || nx >= w-1 || ny <= 0 || ny >= h-1) continue;
                        const int nidx = ny * w + nx;
                        if (visited[nidx]) continue;
                        if (m_cannyNms[nidx] >= low) {
                            visited[nidx] = 1;
                            stack.append(nidx);
                        }
                    }
                }
            }
        }
    }
    return edgeMask;
}

QColor EdgeSharpener::determineEdgeColor(const QImage& /*srcImage*/, int x, int y, int autoInvertRange) {
    if (autoInvertRange == -1) return QColor(0, 0, 0);
    if (autoInvertRange == 0)  return QColor(255, 255, 255);
    // Use pre-built integral image for fast box average
    int r = autoInvertRange;
    int avg = averageGrayInBox(x - r, y - r, x + r, y + r);
    return (avg > 128) ? QColor(0, 0, 0) : QColor(255, 255, 255);
}

QImage EdgeSharpener::processEdgeSharpening(const QImage& srcImage,
    int edgeThreshMin, int edgeThreshMax, int autoInvertRange,
    bool useMetal, const QColor& metalColor,
    bool enablePreFilter, int gaussianKernelSize, double gaussianSigma,
    bool enableDouglasPeucker, double dpTolerance, int dpLineWidth) {
    return processEdgeOperation(srcImage, OperationMode::EdgeEnhance,
        edgeThreshMin, edgeThreshMax, autoInvertRange,
        useMetal, metalColor, enablePreFilter, gaussianKernelSize, gaussianSigma,
        enableDouglasPeucker, dpTolerance, dpLineWidth);
}

QImage EdgeSharpener::processEdgeOperation(const QImage& srcImage,
    OperationMode mode,
    int edgeThreshMin, int edgeThreshMax, int autoInvertRange,
    bool useMetal, const QColor& metalColor,
    bool enablePreFilter, int gaussianKernelSize, double gaussianSigma,
    bool enableDouglasPeucker, double dpTolerance, int dpLineWidth) {

    if (srcImage.isNull()) return QImage();

    QImage working = enablePreFilter
        ? applyGaussianBlur(srcImage, gaussianKernelSize, gaussianSigma)
        : srcImage.convertToFormat(QImage::Format_RGB32);

    buildCacheIfNeeded(working);

    const int w = m_cachedW, h = m_cachedH;

    QImage edgeMask;
    if (mode == OperationMode::StrokeCanny)
        edgeMask = buildCannyMask(edgeThreshMin, edgeThreshMax);
    else
        edgeMask = buildLaplacianMask(edgeThreshMin, edgeThreshMax);

    if (enableDouglasPeucker) {
        auto components = extractConnectedComponents(edgeMask);
        QImage canvas = srcImage.convertToFormat(QImage::Format_ARGB32);
        QPainter painter(&canvas);
        painter.setRenderHint(QPainter::Antialiasing, true);
        for (const auto& comp : components) {
            QVector<QPoint> simplified;
            douglasPeuckerSimplify(comp, dpTolerance, simplified);
            if (simplified.size() >= 2) {
                QColor edgeColor;
                if (useMetal) edgeColor = metalColor.isValid() ? metalColor : QColor(218,165,32);
                else          edgeColor = determineEdgeColor(srcImage, comp.first().x(), comp.first().y(), autoInvertRange);
                QPen pen(edgeColor);
                pen.setWidth(qMax(1, dpLineWidth));
                painter.setPen(pen);
                QVector<QPointF> fp;
                fp.reserve(simplified.size());
                for (const auto& p : simplified) fp.append(QPointF(p));
                painter.drawPolyline(fp.constData(), fp.size());
            }
        }
        painter.end();
        return canvas;
    }

    // Per-pixel coloring via scanLine (no setPixel)
    QImage result = srcImage.convertToFormat(QImage::Format_RGB32);
    for (int y = 1; y < h-1; ++y) {
        const uchar* em = edgeMask.constScanLine(y);
        QRgb* resLine = reinterpret_cast<QRgb*>(result.scanLine(y));
        for (int x = 1; x < w-1; ++x) {
            if (!em[x]) continue;
            QColor edgeColor;
            if (useMetal) {
                edgeColor = metalColor.isValid() ? metalColor : QColor(218,165,32);
            } else {
                if (autoInvertRange == -1)      edgeColor = QColor(0,0,0);
                else if (autoInvertRange == 0)  edgeColor = QColor(255,255,255);
                else {
                    int avg = averageGrayInBox(x - autoInvertRange, y - autoInvertRange,
                                              x + autoInvertRange, y + autoInvertRange);
                    edgeColor = (avg > 128) ? QColor(0,0,0) : QColor(255,255,255);
                }
            }
            resLine[x] = edgeColor.rgb();
        }
    }
    return result;
}

QImage EdgeSharpener::buildEdgeMaskForImage(const QImage& srcImage, OperationMode mode,
    int edgeThreshMin, int edgeThreshMax,
    bool enablePreFilter, int gaussianKernelSize, double gaussianSigma) {
    if (srcImage.isNull()) return QImage();
    QImage working = enablePreFilter
        ? applyGaussianBlur(srcImage, gaussianKernelSize, gaussianSigma)
        : srcImage.convertToFormat(QImage::Format_RGB32);
    buildCacheIfNeeded(working);
    if (mode == OperationMode::StrokeCanny)
        return buildCannyMask(edgeThreshMin, edgeThreshMax);
    return buildLaplacianMask(edgeThreshMin, edgeThreshMax);
}
