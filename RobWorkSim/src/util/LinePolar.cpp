#include "LinePolar.hpp"
#include <rw/common/macros.hpp>
#include <rw/math/Constants.hpp>

LinePolar::LinePolar(
    double rho,
    double theta)
    :
    _rho(rho),
    _theta(theta),
    _normal(cos(theta), sin(theta))
{}

LinePolar LinePolar::make(const P2D& pnt, double theta)
{
    const P2D normal(cos(theta), sin(theta));
    const double rho = dot(pnt, normal);

    if (rho >= 0) {
        return LinePolar(rho, theta);
    } else {
        // Normalize rho and theta.
        const P2D inv_normal = -normal;
        return LinePolar(
            -rho,
            atan2(inv_normal[1], inv_normal[0]));
    }
}

LinePolar LinePolar::make(const P2D& start, const P2D& end)
{
    const P2D diff = end - start;

    RW_ASSERT(diff.norm2() > 1e-14);

    const P2D step = diff / diff.norm2();

    const P2D normal(-step[1], step[0]);
    const double theta = atan2(normal[1], normal[0]);
    return LinePolar::make(start, theta);
}

double LinePolar::dist2(const P2D& pnt)
{
	return fabs(dot(pnt, getNormal()) - _rho);
}

P2D LinePolar::linePoint(const LinePolar& line)
{
    return line.getRho() * line.getNormal();
}

P2D LinePolar::normalProjectionVector(const LinePolar& line, const P2D& pnt)
{
    const P2D p0 = linePoint(line);

    return dot(pnt - p0, line.getNormal()) * line.getNormal();
}

P2D LinePolar::projectionPoint(const LinePolar& line, const P2D& pnt)
{
    return pnt - normalProjectionVector(line, pnt);
}

void LinePolar::print(const LinePolar& line)
{
    printf(
        "(%.2f m, %.2f Deg)",
        line.getRho(),
        line.getTheta() * rw::math::Rad2Deg);
}

typedef boost::numeric::ublas::vector<double> Vec;
Vec LinePolar::toUblas(const LinePolar& line)
{
    Vec vec(2);
    vec(0) = line.getRho();
    vec(1) = line.getTheta();
    return vec;
}

LinePolar LinePolar::lineToLocal(
    const Pose2D& pose,
    const LinePolar& line)
{
    const double ami = line.getTheta();
    const double rho = line.getRho();
    const double x = pose.x();
    const double y = pose.y();
    const double r = sqrt(x*x + y*y);
    const double beta = atan2(y,x);

    const double rho_local = rho - r * cos(ami - beta);
    const double angle_local = ami - pose.theta();
    return LinePolar(rho_local, angle_local);
}
