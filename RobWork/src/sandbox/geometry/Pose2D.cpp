
#include "Pose2D.hpp"


rw::math::Transform2D<> Pose2D::transform(const Pose2D& pose)
{
    return rw::math::Transform2D<>(
            rw::math::Vector2D<>(pose.x(), pose.y()),
            rw::math::Rotation2D<>(pose.theta()));
}

void Pose2D::print(const Pose2D& pose)
{
    std::cout
        << "( x: " << pose.x()
        << ", y: " << pose.y()
        << ", th: " << pose.theta()
        << ")";
}

typedef boost::numeric::ublas::vector<double> Vec;
Vec Pose2D::toUblas(const Pose2D& pose)
{
    Vec vec(3);
    vec(0) = pose.x();
    vec(1) = pose.y();
    vec(2) = pose.theta();
    return vec;
}
