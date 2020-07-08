#include <algorithm>

double getAngularDistance(double a, double b)
{
    // Mod with same sign as divisor (% returns values with same sign as dividend)
    auto mod = [](double a, double n)
    {
        return a - std::floor(a/n) * n;
    };
    return mod(a - b + M_PI, 2*M_PI) - M_PI;
}

int main(int argc, char** argv)
{
    printf("0.3 = %f\n", getAngularDistance(0.3, 4*M_PI));
    printf("-0.3 = %f\n", getAngularDistance(-0.3, 4*M_PI));
    printf("0.3 = %f\n", getAngularDistance(0.3, -4*M_PI));
    printf("-0.3 = %f\n", getAngularDistance(-0.3, -4*M_PI));
    printf("0.6 = %f\n", getAngularDistance(0.3, -(0.3+4*M_PI)));
    printf("0.0 = %f\n", getAngularDistance(0.3, (0.3+4*M_PI)));
    printf("0.0 = %f\n", getAngularDistance(-0.3, -(0.3+4*M_PI)));
    printf("-0.6 = %f\n", getAngularDistance(-0.3, (0.3+4*M_PI)));
    printf("PI = %f\n", getAngularDistance(M_PI, 4*M_PI));
    printf("-PI = %f\n", getAngularDistance(-M_PI, 4*M_PI));
};
