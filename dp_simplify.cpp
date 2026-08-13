// dp_simplify.cpp — iterative Douglas-Peucker (no recursion = no stack overflow)
// extractConnectedComponents with hard safety limits
#include "dp_simplify.h"
#include <QVector>
#include <QPoint>
#include <QImage>
#include <cmath>
#include <algorithm>

// Safety limits — prevent OOM on large images
static const int kMaxComponents   = 2000;   // max number of edge components
static const int kMaxCompPoints   = 4000;   // max points per component
static const int kMaxTotalPoints  = 500000; // total points budget

QVector<QVector<QPoint>> extractConnectedComponents(const QImage& mask) {
    QVector<QVector<QPoint>> components;
    if (mask.isNull()) return components;

    const int w = mask.width();
    const int h = mask.height();
    QVector<char> visited(w * h, 0);

    // Reusable BFS stack
    QVector<int> stack;
    stack.reserve(4096);

    int totalPoints = 0;

    for (int y0 = 1; y0 < h - 1; ++y0) {
        const uchar* row = mask.constScanLine(y0);
        for (int x0 = 1; x0 < w - 1; ++x0) {
            if (!row[x0]) continue;
            const int startIdx = y0 * w + x0;
            if (visited[startIdx]) continue;
            if (components.size() >= kMaxComponents) return components;

            visited[startIdx] = 1;
            stack.clear();
            stack.append(startIdx);

            QVector<QPoint> comp;
            comp.reserve(64);

            while (!stack.isEmpty()) {
                const int idx = stack.takeLast();
                const int cx = idx % w;
                const int cy = idx / w;
                comp.append(QPoint(cx, cy));

                if (comp.size() >= kMaxCompPoints) {
                    // Mark rest of stack as visited and bail
                    for (int i : stack) visited[i] = 1;
                    stack.clear();
                    break;
                }

                // 8-connected neighbors
                const int nx0 = qMax(1, cx - 1);
                const int nx1 = qMin(w - 2, cx + 1);
                const int ny0 = qMax(1, cy - 1);
                const int ny1 = qMin(h - 2, cy + 1);
                for (int ny = ny0; ny <= ny1; ++ny) {
                    const uchar* nrow = mask.constScanLine(ny);
                    for (int nx = nx0; nx <= nx1; ++nx) {
                        if (nx == cx && ny == cy) continue;
                        const int nidx = ny * w + nx;
                        if (!visited[nidx] && nrow[nx]) {
                            visited[nidx] = 1;
                            stack.append(nidx);
                        }
                    }
                }
            }

            if (comp.size() >= 2) {
                totalPoints += comp.size();
                components.append(std::move(comp));
                if (totalPoints >= kMaxTotalPoints) return components;
            }
        }
    }
    return components;
}

static double perpDist(const QPoint& a, const QPoint& b, const QPoint& p) {
    const double dx = b.x() - a.x();
    const double dy = b.y() - a.y();
    const double den = std::sqrt(dx * dx + dy * dy);
    if (den == 0.0) return 0.0;
    return std::abs(dy * p.x() - dx * p.y() + (double)b.x() * a.y() - (double)b.y() * a.x()) / den;
}

// Iterative Douglas-Peucker using an explicit work stack of [start, end] index pairs
void douglasPeuckerSimplify(const QVector<QPoint>& pts, double eps, QVector<QPoint>& out) {
    const int n = pts.size();
    if (n == 0) { out.clear(); return; }
    if (n <= 2)  { out = pts; return; }

    // keep[i] = true means pts[i] survives
    QVector<bool> keep(n, false);
    keep[0]     = true;
    keep[n - 1] = true;

    // Work stack of (lo, hi) index pairs
    QVector<std::pair<int,int>> workStack;
    workStack.reserve(256);
    workStack.append({0, n - 1});

    while (!workStack.isEmpty()) {
        const auto [lo, hi] = workStack.takeLast();
        if (hi <= lo + 1) continue;

        double maxDist = 0.0;
        int    maxIdx  = lo;
        for (int i = lo + 1; i < hi; ++i) {
            double d = perpDist(pts[lo], pts[hi], pts[i]);
            if (d > maxDist) { maxDist = d; maxIdx = i; }
        }

        if (maxDist > eps) {
            keep[maxIdx] = true;
            workStack.append({lo,     maxIdx});
            workStack.append({maxIdx, hi    });
        }
    }

    out.clear();
    out.reserve(n);
    for (int i = 0; i < n; ++i)
        if (keep[i]) out.append(pts[i]);
}
