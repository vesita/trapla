#include "utils/sequence.hpp"

double geometric_sum(double a, double alpha, int n) {
    if (n == 0) return 0;
    if (alpha == 1) return a * n;
    return a * (1 - pow(alpha, n)) / (1 - alpha);
}

double arithmetic_sum(double ab, double ae, int n) {
    if (n == 0) return 0;
    return (ab + ae) * n / 2;
}