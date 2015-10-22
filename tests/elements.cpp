#include <gtest/gtest.h>
#include <spc/elements/Plane.h>
#include <spc/elements/Attitude.h>

#define GTEST_PREC 1e-6

TEST(Plane, RightDistance) {

    spc::Plane p({0,0,1}, {0,1,1});
    EXPECT_EQ(p.distanceTo({0,0,0}), -1);
    EXPECT_EQ(p.distanceTo({0,0,-1}), -2);

    p.setNormal({0,1,1});
    EXPECT_NEAR(p.distanceTo({0,0,0}), -sqrt(2), GTEST_PREC);
    EXPECT_NEAR(p.distanceTo({0,2,2}), sqrt(2), GTEST_PREC);

}

TEST(Attitude, Plane2DipAndDipAngle)
{
    spc::Attitude att(45, 0);

//    LOG(INFO) << att.getNormal();
    ASSERT_TRUE(att.getNormal().isApprox(Eigen::Vector3f({0, sqrt(0.5), sqrt(0.5)}), GTEST_PREC));


    att = spc::Attitude(45, 45);
//    LOG(INFO) << att.getNormal();
    ASSERT_TRUE(att.getNormal().isApprox(Eigen::Vector3f({0.5, 0.5, sqrt(0.5)}), GTEST_PREC));
}

TEST(Attitude, NegativeEqualPositiveDips)
{
    spc::Attitude att1(45, 45);
    spc::Attitude att2(45, -315);

    ASSERT_TRUE(att1.getNormal().isApprox(att2.getNormal(), GTEST_PREC));

}


TEST(ElementBase,  DtiClass_system)
{
    spc::Attitude::Ptr a(new spc::Attitude);
    spc::ElementBase::Ptr el_ptr;
    el_ptr = a;

    EXPECT_TRUE(el_ptr->isA(&spc::Attitude::Type));

    EXPECT_EQ(a->getType()->getClassName(), el_ptr->getType()->getClassName());
}
