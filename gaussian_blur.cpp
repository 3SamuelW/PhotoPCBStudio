// gaussian_blur.cpp — separable Gaussian blur, scanLine-based (no pixel() calls)
#include "gaussian_blur.h"
#include <QVector>
#include <cmath>

static int clampOddKernelSize(int kernelSize) {
    if (kernelSize < 3) kernelSize = 3;
    if (kernelSize > 11) kernelSize = 11;
    if ((kernelSize % 2) == 0) --kernelSize;
    if (kernelSize < 3) kernelSize = 3;
    return kernelSize;
}

static QVector<double> buildGaussian1DKernel(int kernelSize, double sigma) {
    kernelSize = clampOddKernelSize(kernelSize);
    int radius = kernelSize / 2;
    QVector<double> w(kernelSize);
    const double twoSigma2 = 2.0 * sigma * sigma;
    double sum = 0.0;
    for (int i = -radius; i <= radius; ++i) {
        double v = std::exp(-(i * i) / twoSigma2);
        w[i + radius] = v;
        sum += v;
    }
    for (int i = 0; i < kernelSize; ++i) w[i] /= sum;
    return w;
}

QImage applyGaussianBlur(const QImage& srcImage, int kernelSize, double sigma) {
    kernelSize = clampOddKernelSize(kernelSize);
    if (srcImage.isNull() || kernelSize <= 1) return srcImage;

    const QVector<double> weights = buildGaussian1DKernel(kernelSize, sigma);
    const int radius = kernelSize / 2;

    // Always work in RGB32 (no alpha needed, avoids format issues)
    QImage src = srcImage.convertToFormat(QImage::Format_RGB32);
    QImage temp(src.size(), QImage::Format_RGB32);
    QImage dst(src.size(), QImage::Format_RGB32);

    const int w = src.width();
    const int h = src.height();

    // Build row-pointer cache to avoid repeated scanLine calls in inner loop
    QVector<const QRgb*> rows(h);
    for (int y = 0; y < h; ++y)
        rows[y] = reinterpret_cast<const QRgb*>(src.constScanLine(y));

    // Horizontal pass: src -> temp
    for (int y = 0; y < h; ++y) {
        const QRgb* in = rows[y];
        QRgb* out = reinterpret_cast<QRgb*>(temp.scanLine(y));
        for (int x = 0; x < w; ++x) {
            double r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sx = qBound(0, x + k, w - 1);
                const QRgb px = in[sx];
                const double wt = weights[k + radius];
                r += qRed(px)   * wt;
                g += qGreen(px) * wt;
                b += qBlue(px)  * wt;
            }
            out[x] = qRgb(static_cast<int>(r + 0.5),
                          static_cast<int>(g + 0.5),
                          static_cast<int>(b + 0.5));
        }
    }

    // Build row-pointer cache for temp
    QVector<const QRgb*> trows(h);
    for (int y = 0; y < h; ++y)
        trows[y] = reinterpret_cast<const QRgb*>(temp.constScanLine(y));

    // Vertical pass: temp -> dst
    for (int x = 0; x < w; ++x) {
        for (int y = 0; y < h; ++y) {
            double r = 0, g = 0, b = 0;
            for (int k = -radius; k <= radius; ++k) {
                const int sy = qBound(0, y + k, h - 1);
                const QRgb px = trows[sy][x];
                const double wt = weights[k + radius];
                r += qRed(px)   * wt;
                g += qGreen(px) * wt;
                b += qBlue(px)  * wt;
            }
            reinterpret_cast<QRgb*>(dst.scanLine(y))[x] =
                qRgb(static_cast<int>(r + 0.5),
                     static_cast<int>(g + 0.5),
                     static_cast<int>(b + 0.5));
        }
    }

    return dst;
}
