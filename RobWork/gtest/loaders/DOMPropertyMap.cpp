#include <gtest/gtest.h>

#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>

#include <rw/trajectory/Path.hpp>

#include <rw/math/Math.hpp>

#include <algorithm>
#include <sstream>

using namespace rw::math;
using namespace rw::trajectory;

namespace {
    /* Declaring the functions for recursive use */
    void comparePropertyMaps(const rw::common::PropertyMap &a, const rw::common::PropertyMap &b);

    void compareProperties(const rw::common::PropertyBase::Ptr a, const rw::common::PropertyBase::Ptr b) {
        EXPECT_EQ(a->getIdentifier(), b->getIdentifier());
        EXPECT_EQ(a->getDescription(), b->getDescription());
        ASSERT_EQ(a->getType().getId(), b->getType().getId());

        switch (a->getType().getId()) {
        case rw::common::PropertyType::PropertyMap: {
            const rw::common::Property<rw::common::PropertyMap>* pa = rw::common::toProperty<rw::common::PropertyMap>(a);
            const rw::common::Property<rw::common::PropertyMap>* pb = rw::common::toProperty<rw::common::PropertyMap>(b);
            comparePropertyMaps(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::String: {
            const rw::common::Property<std::string>* pa = rw::common::toProperty<std::string>(a);
            const rw::common::Property<std::string>* pb = rw::common::toProperty<std::string>(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Float: {
            const rw::common::Property<float>* pa = rw::common::toProperty<float>(a);
            const rw::common::Property<float>* pb = rw::common::toProperty<float>(b);
            EXPECT_FLOAT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Double: {
            const rw::common::Property<double>* pa = rw::common::toProperty<double>(a);
            const rw::common::Property<double>* pb = rw::common::toProperty<double>(b);
            EXPECT_DOUBLE_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Int: {
            const rw::common::Property<int>* pa = rw::common::toProperty<int>(a);
            const rw::common::Property<int>* pb = rw::common::toProperty<int>(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Bool: {
            const rw::common::Property<bool>* pa = rw::common::toProperty<bool>(a);
            const rw::common::Property<bool>* pb = rw::common::toProperty<bool>(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Vector3D: {
            const rw::common::Property<Vector3D<> >* pa = rw::common::toProperty<Vector3D<> >(a);
            const rw::common::Property<Vector3D<> >* pb = rw::common::toProperty<Vector3D<> >(b);
            EXPECT_DOUBLE_EQ(pa->getValue()[0],pb->getValue()[0]);
            EXPECT_DOUBLE_EQ(pa->getValue()[1],pb->getValue()[1]);
            EXPECT_DOUBLE_EQ(pa->getValue()[2],pb->getValue()[2]);
            break;
        }
        case rw::common::PropertyType::Vector2D: {
            const rw::common::Property<Vector2D<> >* pa = rw::common::toProperty<Vector2D<> >(a);
            const rw::common::Property<Vector2D<> >* pb = rw::common::toProperty<Vector2D<> >(b);
            EXPECT_DOUBLE_EQ(pa->getValue()[0],pb->getValue()[0]);
            EXPECT_DOUBLE_EQ(pa->getValue()[1],pb->getValue()[1]);
            break;
        }
        case rw::common::PropertyType::Q: {
            const rw::common::Property<Q>* pa = rw::common::toProperty<Q>(a);
            const rw::common::Property<Q>* pb = rw::common::toProperty<Q>(b);
            ASSERT_EQ(pa->getValue().size(), pb->getValue().size());
            for (std::size_t i = 0; i < pa->getValue().size(); i++) {
            	EXPECT_DOUBLE_EQ(pa->getValue()[i], pb->getValue()[i]);
            }
            break;
        }
        case rw::common::PropertyType::Transform3D: {
            const rw::common::Property<Transform3D<> >* pa = rw::common::toProperty<Transform3D<> >(a);
            const rw::common::Property<Transform3D<> >* pb = rw::common::toProperty<Transform3D<> >(b);
            EXPECT_TRUE(pa->getValue().equal(pb->getValue()));
            break;
        }
        case rw::common::PropertyType::Rotation3D: {
            const rw::common::Property<Rotation3D<> >* pa = rw::common::toProperty<Rotation3D<> >(a);
            const rw::common::Property<Rotation3D<> >* pb = rw::common::toProperty<Rotation3D<> >(b);
            EXPECT_TRUE(pa->getValue().equal(pb->getValue()));
            break;
        }
        case rw::common::PropertyType::RPY: {
            const rw::common::Property<RPY<> >* pa = rw::common::toProperty<RPY<> >(a);
            const rw::common::Property<RPY<> >* pb = rw::common::toProperty<RPY<> >(b);
            EXPECT_DOUBLE_EQ(pa->getValue()[0],pb->getValue()[0]);
            EXPECT_DOUBLE_EQ(pa->getValue()[1],pb->getValue()[1]);
            EXPECT_DOUBLE_EQ(pa->getValue()[2],pb->getValue()[2]);
            break;
        }
        case rw::common::PropertyType::EAA: {
            const rw::common::Property<EAA<> >* pa = rw::common::toProperty<EAA<> >(a);
            const rw::common::Property<EAA<> >* pb = rw::common::toProperty<EAA<> >(b);
            EXPECT_DOUBLE_EQ(pa->getValue()[0],pb->getValue()[0]);
            EXPECT_DOUBLE_EQ(pa->getValue()[1],pb->getValue()[1]);
            EXPECT_DOUBLE_EQ(pa->getValue()[2],pb->getValue()[2]);
            break;
        }
        case rw::common::PropertyType::Quaternion: {
            const rw::common::Property<Quaternion<> >* pa = rw::common::toProperty<Quaternion<> >(a);
            const rw::common::Property<Quaternion<> >* pb = rw::common::toProperty<Quaternion<> >(b);
            EXPECT_NEAR(pa->getValue()(0),pb->getValue()(0),1e-6);
            EXPECT_NEAR(pa->getValue()(1),pb->getValue()(1),1e-6);
            EXPECT_NEAR(pa->getValue()(2),pb->getValue()(2),1e-6);
            EXPECT_NEAR(pa->getValue()(3),pb->getValue()(3),1e-6);
            break;
        }
        case rw::common::PropertyType::Rotation2D: {
            const rw::common::Property<Rotation2D<> >* pa = rw::common::toProperty<Rotation2D<> >(a);
            const rw::common::Property<Rotation2D<> >* pb = rw::common::toProperty<Rotation2D<> >(b);
            EXPECT_DOUBLE_EQ(pa->getValue()(0,0),pb->getValue()(0,0));
            EXPECT_DOUBLE_EQ(pa->getValue()(0,1),pb->getValue()(0,1));
            EXPECT_DOUBLE_EQ(pa->getValue()(1,0),pb->getValue()(1,0));
            EXPECT_DOUBLE_EQ(pa->getValue()(1,1),pb->getValue()(1,1));
            break;
        }
        case rw::common::PropertyType::VelocityScrew6D: {
            const rw::common::Property<VelocityScrew6D<> >* pa = rw::common::toProperty<VelocityScrew6D<> >(a);
            const rw::common::Property<VelocityScrew6D<> >* pb = rw::common::toProperty<VelocityScrew6D<> >(b);
            for (std::size_t i = 0; i < 6; i++) {
            	EXPECT_DOUBLE_EQ(pa->getValue()[i],pb->getValue()[i]);
            }
            break;
        }
        case rw::common::PropertyType::QPath: {
            /* Doing verbose comparison, else the operator== and operator!= could be implemented for rw::trajectory::Path */
            const rw::common::Property<QPath>* pa = rw::common::toProperty<QPath>(a);
            const rw::common::Property<QPath>* pb = rw::common::toProperty<QPath>(b);
            EXPECT_EQ(pa->getValue().size(), pb->getValue().size());
            
            std::size_t maxIndex = std::max(pa->getValue().size(), pb->getValue().size());
            for (std::size_t index = 0; index < maxIndex; ++index) {
                if (index >= pa->getValue().size()) {
                    EXPECT_EQ(Q(), pb->getValue().at(index));
                } else if (index >= pb->getValue().size()) {
                    EXPECT_EQ(pa->getValue().at(index), Q());
                } else {
                	const Q& qa = pa->getValue().at(index);
                	const Q& qb = pb->getValue().at(index);
                	ASSERT_EQ(qa.size(),qb.size());
                	for (std::size_t i = 0; i < qa.size(); i++) {
                		EXPECT_DOUBLE_EQ(qa[i], qb[i]);
                	}
                }
            }
            break;
        }
        case rw::common::PropertyType::Transform3DPath: {
            /* Doing verbose comparison, else the operator== and operator!= could be implemented for rw::trajectory::Path */
            const rw::common::Property<Transform3DPath>* pa = rw::common::toProperty<Transform3DPath>(a);
            const rw::common::Property<Transform3DPath>* pb = rw::common::toProperty<Transform3DPath>(b);
            EXPECT_EQ(pa->getValue().size(), pb->getValue().size());
            
            std::size_t maxIndex = std::max(pa->getValue().size(), pb->getValue().size());
            for (std::size_t index = 0; index < maxIndex; ++index) {
                if (index >= pa->getValue().size()) {
                    EXPECT_EQ(Transform3D<>(), pb->getValue().at(index));
                } else if (index >= pb->getValue().size()) {
                    EXPECT_EQ(pa->getValue().at(index), Transform3D<>());
                } else {
                    EXPECT_TRUE(pa->getValue().at(index).equal(pb->getValue().at(index)));
                }
            }
            break;
        }
        case rw::common::PropertyType::StringList: {
            const rw::common::Property<std::vector<std::string> >* pa = rw::common::toProperty<std::vector<std::string> >(a);
            const rw::common::Property<std::vector<std::string> >* pb = rw::common::toProperty<std::vector<std::string> >(b);
            EXPECT_EQ(pa->getValue().size(), pb->getValue().size());
            
            std::size_t maxIndex = std::max(pa->getValue().size(), pb->getValue().size());
            for (std::size_t index = 0; index < maxIndex; ++index) {
                if (index >= pa->getValue().size()) {
                    EXPECT_EQ(std::string(""), pb->getValue().at(index));
                } else if (index >= pb->getValue().size()) {
                    EXPECT_EQ(pa->getValue().at(index), std::string(""));
                } else {
                    EXPECT_EQ(pa->getValue().at(index), pb->getValue().at(index));
                }
            }
            break;
        }
        case rw::common::PropertyType::IntList: {
            const rw::common::Property<std::vector<int> >* pa = rw::common::toProperty<std::vector<int> >(a);
            const rw::common::Property<std::vector<int> >* pb = rw::common::toProperty<std::vector<int> >(b);
            EXPECT_EQ(pa->getValue().size(), pb->getValue().size());
            
            std::size_t maxIndex = std::max(pa->getValue().size(), pb->getValue().size());
            for (std::size_t index = 0; index < maxIndex; ++index) {
                if (index >= pa->getValue().size()) {
                    EXPECT_EQ(0, pb->getValue().at(index));
                } else if (index >= pb->getValue().size()) {
                    EXPECT_EQ(pa->getValue().at(index), 0);
                } else {
                    EXPECT_EQ(pa->getValue().at(index), pb->getValue().at(index));
                }
            }
            break;
        }
        case rw::common::PropertyType::DoubleList: {
            const rw::common::Property<std::vector<double> >* pa = rw::common::toProperty<std::vector<double> >(a);
            const rw::common::Property<std::vector<double> >* pb = rw::common::toProperty<std::vector<double> >(b);
            EXPECT_EQ(pa->getValue().size(), pb->getValue().size());
            
            std::size_t maxIndex = std::max(pa->getValue().size(), pb->getValue().size());
            for (std::size_t index = 0; index < maxIndex; ++index) {
                if (index >= pa->getValue().size()) {
                    EXPECT_EQ(Math::NaN(), pb->getValue().at(index));
                } else if (index >= pb->getValue().size()) {
                    EXPECT_EQ(pa->getValue().at(index), Math::NaN());
                } else {
                    EXPECT_EQ(pa->getValue().at(index), pb->getValue().at(index));
                }
            }
            break;
        }
        default: {
            FAIL() << "The property type (" << a->getType().getId() << ") has no comparison implementation!";
            break;
        }
        }
    }

    void comparePropertyMaps(const rw::common::PropertyMap &a, const rw::common::PropertyMap &b) {
        ASSERT_EQ(a.size(), b.size());

        rw::common::PropertyMap::iterator it;
        for (it = a.getProperties().first; it != a.getProperties().second; ++it) {
            rw::common::PropertyBase::Ptr pa, pb;
            pa = *it;
            pb = b.findPropertyBase(pa->getIdentifier());
            ASSERT_FALSE(pb == NULL);

            compareProperties(pa, pb);
        }
    }        

    void testSaverAndLoader(const rw::common::PropertyMap &map) {
        {
            rw::common::PropertyMap mapIn;
            /* Currently just saving to a file in the current directory. Not even cleaning it up or ensuring that it is emptied before writing new things to it. Would probably be more ideal to place the file in $TMP and/or remove it between function invocations. */
            const std::string file("PropertyMap_stringList.xml");
            SCOPED_TRACE("Serialization to file");
            ASSERT_NO_THROW(rw::loaders::DOMPropertyMapSaver::save(map, file));
            ASSERT_NO_THROW({
                    mapIn = rw::loaders::DOMPropertyMapLoader::load(file);
                });
        
            comparePropertyMaps(map, mapIn);
        }
        {
            rw::common::PropertyMap mapIn;
            /* Test iostreams */
            std::stringstream mapStream;
            SCOPED_TRACE("Serialization to output stream");
            ASSERT_NO_THROW(rw::loaders::DOMPropertyMapSaver::write(map, mapStream));
            ASSERT_NO_THROW({
                    mapIn = rw::loaders::DOMPropertyMapLoader::load(mapStream);
                });
            
            comparePropertyMaps(map, mapIn);
        }
    }
}

TEST(DOMPropertyMapSaveAndLoad, PropertyMap) {
    std::vector<rw::common::PropertyMap> items;
    {
        rw::common::PropertyMap item;

        items.push_back(item);
    }
    {
        rw::common::PropertyMap item;
        rw::common::PropertyMap m;

        EXPECT_TRUE(item.add(std::string("int"), std::string("The description of an Int"), 100));
        EXPECT_TRUE(m.add(std::string("int"), std::string("The description of an Int"), 1001));
        EXPECT_TRUE(item.add(std::string("propertymap"), std::string("The description of a PropertyMap"), m));
        EXPECT_TRUE(item.add(std::string("double"), std::string("The description of a Double"), 0.4));

        items.push_back(item);
    }

    for (std::vector<rw::common::PropertyMap>::const_iterator it = items.begin(); it != items.end(); ++it) {
        testSaverAndLoader(*it);
    }
}


TEST(DOMPropertyMapSaveAndLoad, String) {
    std::vector<std::string> items;
    items.push_back(std::string("NoneEOUvuWIrPQZTidCXgLYHby"));
    items.push_back(std::string("NoneMAsnKWlhLzdZDBvSVCxRakwYpEyQiHIbJtXOoeg"));
    items.push_back(std::string("NonePeYs"));
    items.push_back(std::string("NonewLnYlA"));
    items.push_back(std::string("NoneF"));
    items.push_back(std::string("NoneBNjgquOWL"));
    items.push_back(std::string("NoneFWJBumeQgwODrIhbRVTkaSLNUynxotCzdMvZKXGEPpsY"));
    items.push_back(std::string("NoneDhGlfQRzyJmkunjgUocCbdYXIEAZNiqvrMws"));
    items.push_back(std::string("NoneRYBDnQrCfAqGTehuiSwv"));
    items.push_back(std::string("This is a manually written string!"));
    for (std::vector<std::string>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a String"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Float) {
    std::vector<float> items;
    items.push_back(0.3458f);
    items.push_back(-0.0f);
    items.push_back(0.0f);
    items.push_back(-1.22f);
    items.push_back(-123.4545f);
    items.push_back(6213.2f);
    items.push_back(0.0234f);
    for (std::vector<float>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Float"), *it));
        
        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Double) {
    std::vector<double> items;
    items.push_back(-0.1823);
    items.push_back(-0.0);
    items.push_back(0.0);
    items.push_back(-1.86);
    items.push_back(1.982);
    items.push_back(-1235.2);
    items.push_back(7823.9332);
    for (std::vector<double>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Double"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Int) {
    std::vector<int> items;
    items.push_back(-34598);
    items.push_back(-1938484);
    items.push_back(233323);
    items.push_back(0);
    items.push_back(1);
    items.push_back(45588);
    items.push_back(510);
    for (std::vector<int>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of an Int"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Bool) {
    std::vector<bool> items;
    items.push_back(false);
    items.push_back(true);
    for (std::vector<bool>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Bool"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Vector3D) {
    std::vector<Vector3D<> > items;
    items.push_back(Vector3D<>(-2.7, 3.0, -1003.2));
    items.push_back(Vector3D<>(1.9, -2.0, 0.1));
    items.push_back(Vector3D<>(0.0, 0.0, 0.0));
    items.push_back(Vector3D<>(12.0, 12340.0, 98822.0));
    for (std::vector<Vector3D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Vector3D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Vector2D) {
    std::vector<Vector2D<> > items;
    items.push_back(Vector2D<>(-2.7, 3.0));
    items.push_back(Vector2D<>(1.9, 2.0));
    items.push_back(Vector2D<>(0.0, 0.0));
    for (std::vector<Vector2D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Vector2D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Q) {
    std::vector<Q> items;
    items.push_back(Q(3, 0.86, -1.2, 1.4));
    items.push_back(Q(7, 4.0, -0.12, -3.1, -2.2, 2.1, -3.1, -2.7));
    items.push_back(Q(5, -1.8, 2.2, 0.45, 5.1, -0.44));
    items.push_back(Q(9, 6.0, 4.0, 1.9, 5.3, -0.62, -3.2, 4.2, 4.0, 0.2));
    items.push_back(Q(3, 1.4, -4.0, 0.25));
    items.push_back(Q(5, -0.82, 4.7, 4.7, 5.0, -2.5));
    items.push_back(Q(4, 2.2, -4.5, -4.1, -0.41));
    items.push_back(Q(7, -1.4, 0.89, 5.5, -3.6, 0.63, -3.6, -0.87));
    items.push_back(Q(2, 4.1, 2.0));
    items.push_back(Q(1, 5.2));
    for (std::vector<Q>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of an Q"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Transform3D) {
    std::vector<Transform3D<> > items;
    items.push_back(Transform3D<>(Vector3D<>(0.296485, -0.950844, -0.0893953), Rotation3D<>(-0.25984695, -0.17568155, 0.94953439, 0.49147629, -0.87048585, -0.026560061, 0.83122237, 0.45977209, 0.31253641)));
    items.push_back(Transform3D<>(Vector3D<>(0.347301, 0.809361, -0.47362), Rotation3D<>(0.10857248, 0.031855376, 0.993578, -0.37274339, -0.92526036, 0.070396274, 0.92156084, -0.37799273, -0.08858393)));
    items.push_back(Transform3D<>(Vector3D<>(-0.998239, -0.0229768, 0.0546927), Rotation3D<>(0.7102421, -0.1698931, -0.68314895, 0.60260283, -0.35492908, 0.71476931, -0.3639038, -0.91932676, -0.14970751)));
    items.push_back(Transform3D<>(Vector3D<>(-0.167873, 0.902236, -0.397227), Rotation3D<>(-0.43729678, -0.82973847, 0.34685098, -0.23221789, 0.47678372, 0.84779251, -0.86881896, 0.29019193, -0.4011761)));
    items.push_back(Transform3D<>(Vector3D<>(-0.0504132, -0.856888, -0.513031), Rotation3D<>(-0.095453794, 0.36948604, 0.92432064, 0.24084522, -0.89239941, 0.38159779, 0.96585825, 0.25904316, -0.0038060374)));
    items.push_back(Transform3D<>(Vector3D<>(0.182764, 0.467202, 0.865054), Rotation3D<>(-0.67616195, 0.039425984, 0.73569736, -0.70164803, 0.27010537, -0.65934295, -0.22471105, -0.96202322, -0.15497181)));
    items.push_back(Transform3D<>(Vector3D<>(-0.392463, 0.883221, 0.256696), Rotation3D<>(0.674934, -0.73590954, -0.053863268, -0.62684432, -0.5333322, -0.56799909, 0.38926893, 0.41712578, -0.82126475)));
    items.push_back(Transform3D<>(Vector3D<>(-0.118084, 0.44222, -0.889099), Rotation3D<>(-0.71277384, -0.23857637, 0.65957166, 0.33265399, 0.71289515, 0.61735066, -0.61749071, 0.65944054, -0.42876963)));
    items.push_back(Transform3D<>(Vector3D<>(0.0857885, -0.990521, -0.10728), Rotation3D<>(-0.17678865, 0.8856797, 0.42932184, 0.75682682, 0.40119343, -0.51600097, -0.62925268, 0.23369917, -0.7412326)));
    for (std::vector<Transform3D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Transform3D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Rotation3D) {
    std::vector<Rotation3D<> > items;
    items.push_back(Rotation3D<>(-0.80117532, -0.21512602, 0.55842538, 0.1676601, 0.81508895, 0.55454495, -0.5744634, 0.53791338, -0.6169611));
    items.push_back(Rotation3D<>(0.0027003767, -0.93557444, 0.3531192, 0.6786974, 0.26105192, 0.68645592, -0.73441306, 0.23780739, 0.63567689));
    items.push_back(Rotation3D<>(-0.52334027, 0.23196375, 0.81994377, -0.34581175, -0.93725135, 0.04443124, 0.77879984, -0.26029353, 0.57071717));
    items.push_back(Rotation3D<>(-0.13875231, -0.85541813, 0.49900664, 0.77736297, -0.40625753, -0.48027246, 0.61355897, 0.32127037, 0.72133955));
    items.push_back(Rotation3D<>(0.35755596, 0.60382316, -0.71242637, 0.2683868, -0.79711527, -0.54090273, -0.89449554, 0.0021971638, -0.44707148));
    items.push_back(Rotation3D<>(-0.49590912, -0.83547488, 0.2367612, 0.64985797, -0.17621344, 0.73934663, -0.57598503, 0.52050989, 0.63032587));
    items.push_back(Rotation3D<>(-0.20145729, 0.97889118, 0.034453107, -0.73229358, -0.17388078, 0.65841597, 0.65050832, 0.10741291, 0.75186528));
    items.push_back(Rotation3D<>(-0.97428403, 0.014469281, 0.22485834, 0.01021749, 0.99974655, -0.020060972, -0.22509161, -0.017247597, -0.97418493));
    items.push_back(Rotation3D<>(-0.44357068, -0.57814221, -0.68483329, 0.039335173, -0.77594071, 0.62957824, -0.89537579, 0.25232441, 0.36692587));
    for (std::vector<Rotation3D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Rotation3D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, RPY) {
    std::vector<RPY<> > items;
    items.push_back(RPY<>(-2.7, 3.0, -5.9));
    items.push_back(RPY<>(1.9, 2.0, 1.2));
    items.push_back(RPY<>(0.0, 0.0, 0.0));
    items.push_back(RPY<>(2.7, 4.0, -0.12));
    items.push_back(RPY<>(-3.1, -2.2, 2.1));
    items.push_back(RPY<>(-3.1, -2.7, 0.32));
    items.push_back(RPY<>(-2.8, 0.5, 5.1));
    items.push_back(RPY<>(-0.44, 5.6, 4.0));
    items.push_back(RPY<>(1.9, 5.3, -0.62));
    items.push_back(RPY<>(-3.2, 4.2, 4.0));
    items.push_back(RPY<>(0.2, -1.2, 1.4));
    items.push_back(RPY<>(-4.0, 0.25, 0.68));
    for (std::vector<RPY<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of an RPY"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, EAA) {
    std::vector<EAA<> > items;
    items.push_back(EAA<>(0.51, -1.5, 5.5));
    items.push_back(EAA<>(-0.66, -1.4, 4.1));
    items.push_back(EAA<>(4.3, 4.9, -4.6));
    items.push_back(EAA<>(4.5, -4.3, -1.6));
    items.push_back(EAA<>(1.9, 2.6, 2.9));
    items.push_back(EAA<>(2.4, -3.2, -1.8));
    items.push_back(EAA<>(1.6, 3.9, -3.6));
    items.push_back(EAA<>(-4.7, -1.4, 1.3));
    items.push_back(EAA<>(-2.9, 3.2, 1.6));
    for (std::vector<EAA<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of an EAA"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Quaternion) {
    std::vector<Quaternion<> > items;
    items.push_back(Quaternion<>(5.5, 1.2, 1.9, 5.6));
    items.push_back(Quaternion<>(5.1, 5.8, -3.0, -0.98));
    items.push_back(Quaternion<>(3.3, 1.9, -0.73, 1.1));
    items.push_back(Quaternion<>(4.5, 3.2, -1.9, -0.84));
    items.push_back(Quaternion<>(1.4, 4.4, 3.2, -1.9));
    items.push_back(Quaternion<>(0.91, 4.7, -4.6, 0.27));
    items.push_back(Quaternion<>(-1.1, -2.2, -3.0, 3.7));
    items.push_back(Quaternion<>(3.1, 2.7, 1.7, 0.68));
    items.push_back(Quaternion<>(-0.44, -4.0, -4.4, -4.7));
    // Normalize the quaternions as they are normalized within the serialization and deserialization process
    for (std::vector<Quaternion<> >::iterator it = items.begin(); it != items.end(); ++it) {
    	it->normalize();
    }
    for (std::vector<Quaternion<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Quaternion"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Rotation2D) {
    std::vector<Rotation2D<> > items;
    items.push_back(Rotation2D<>(-1.1, -2.8, 2.6, 0.36));
    items.push_back(Rotation2D<>(5.7, 4.7, -2.8, 4.9));
    items.push_back(Rotation2D<>(4.5, 0.81, -1.1, -4.3));
    items.push_back(Rotation2D<>(5.9, -4.5, 0.53, -3.1));
    items.push_back(Rotation2D<>(-0.1, -2.6, -4.8, -2.9));
    items.push_back(Rotation2D<>(4.9, 1.9, 4.7, 5.9));
    items.push_back(Rotation2D<>(-0.87, 1.3, 4.2, -1.5));
    items.push_back(Rotation2D<>(2.3, 2.0, -4.6, 0.45));
    items.push_back(Rotation2D<>(1.7, 0.58, 5.2, 4.7));
    for (std::vector<Rotation2D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Rotation2D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, VelocityScrew6D) {
    std::vector<VelocityScrew6D<> > items;
    items.push_back(VelocityScrew6D<>(2.8, 2.1, -0.62, 4.6, 2.7, -4.2));
    items.push_back(VelocityScrew6D<>(2.7, 3.5, -1.8, -0.26, -4.1, -1.7));
    items.push_back(VelocityScrew6D<>(5.3, 5.2, 5.7, 0.049, -2.0, 5.7));
    items.push_back(VelocityScrew6D<>(-0.15, -4.3, -3.6, -2.2, -0.75, 2.5));
    items.push_back(VelocityScrew6D<>(4.6, -4.5, -4.7, -1.5, 4.9, 2.7));
    items.push_back(VelocityScrew6D<>(-0.95, 2.0, -3.0, -4.2, 2.5, 2.7));
    items.push_back(VelocityScrew6D<>(-2.1, 5.0, -3.9, 4.4, 0.22, -3.7));
    items.push_back(VelocityScrew6D<>(0.96, -2.2, -3.9, 1.8, -3.8, 1.8));
    items.push_back(VelocityScrew6D<>(5.2, 5.9, -2.1, -2.7, 0.58, 1.8));
    for (std::vector<VelocityScrew6D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a VelocityScrew6D"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, QPath) {
    std::vector<QPath> items;
    std::vector<Q> qitems;

    qitems.clear();
    qitems.push_back(Q(7, -2.1, -0.042, -3.2, 0.34, -5.0, 1.5, -3.5));
    qitems.push_back(Q(2, -0.91, 5.4));
    qitems.push_back(Q(8, 5.8, -1.1, -1.6, -2.9, -2.2, -0.17, 5.4, -2.3));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(4, 1.1, -4.3, 2.2, -2.0));
    qitems.push_back(Q(4, -1.2, -1.8, 2.8, -1.8));
    qitems.push_back(Q(1, 1.6));
    qitems.push_back(Q(6, -4.1, -0.92, 2.5, 5.3, -1.5, -4.7));
    qitems.push_back(Q(4, -3.7, -1.3, -4.1, 2.0));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(7, -0.97, -1.2, -4.1, -2.3, -1.3, 4.2, 4.1));
    qitems.push_back(Q(5, 5.2, 0.83, 2.7, -4.5, -5.0));
    qitems.push_back(Q(3, -2.4, 2.2, -2.3));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(5, 1.1, -0.0017, 3.1, 1.1, 0.15));
    qitems.push_back(Q(8, 5.3, -4.6, -2.5, 0.9, 2.9, -4.2, 3.1, -1.5));
    qitems.push_back(Q(8, 5.1, 1.2, 5.4, 4.0, 2.8, -0.52, -1.5, 4.7));
    qitems.push_back(Q(1, 0.36));
    qitems.push_back(Q(4, -4.0, -0.12, 1.8, -2.7));
    qitems.push_back(Q(1, -1.1));
    qitems.push_back(Q(5, 2.3, 1.5, -1.7, -3.9, -4.0));
    qitems.push_back(Q(5, 0.5, 1.4, 2.4, 3.9, -0.5));
    qitems.push_back(Q(3, 5.9, 3.1, 5.8));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(3, 3.4, 0.32, -3.4));
    qitems.push_back(Q(4, -2.6, 2.3, -2.2, 1.9));
    qitems.push_back(Q(2, 2.8, -2.9));
    qitems.push_back(Q(8, 4.4, -0.47, 5.5, 3.9, 2.7, 4.6, -0.39, -0.19));
    qitems.push_back(Q(7, -3.9, 1.1, 4.0, -2.6, 0.88, 0.95, -0.45));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(0));
    qitems.push_back(Q(7, -0.42, 1.8, 4.7, 3.4, -2.6, 4.6, -2.9));
    qitems.push_back(Q(1, -1.1));
    qitems.push_back(Q(8, 4.1, 3.7, -1.3, 5.8, -0.81, 1.3, 1.4, -2.2));
    qitems.push_back(Q(5, 2.6, 2.6, 0.14, 1.8, 5.1));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(1, -1.3));
    qitems.push_back(Q(8, 4.9, -2.0, -4.5, -0.38, -2.3, 3.9, -4.6, -0.95));
    qitems.push_back(Q(7, -3.9, 4.3, 2.6, -2.2, -4.2, 0.036, 1.7));
    qitems.push_back(Q(9, -1.2, 3.1, -3.4, 1.2, 5.7, 3.8, 0.17, -0.16, 4.5));
    qitems.push_back(Q(1, 5.6));
    qitems.push_back(Q(7, 0.82, 3.2, -0.51, -3.7, 2.2, 2.9, -4.4));
    qitems.push_back(Q(2, 2.9, 4.2));
    qitems.push_back(Q(3, -0.99, 4.9, -4.0));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(8, -0.48, 2.3, 2.6, -1.0, 4.9, 1.2, -0.51, 2.8));
    qitems.push_back(Q(3, -2.8, 3.3, -4.6));
    qitems.push_back(Q(1, 5.0));
    qitems.push_back(Q(0));
    qitems.push_back(Q(4, 1.2, -2.1, -4.2, 4.5));

    items.push_back(qitems);
    qitems.clear();

    qitems.push_back(Q(2, -3.6, -0.58));
    qitems.push_back(Q(5, 1.0, -2.1, 5.2, -1.8, -3.8));
    qitems.push_back(Q(0));
    qitems.push_back(Q(7, -3.1, -1.3, -2.3, -3.5, 4.0, 3.8, 5.1));

    items.push_back(qitems);
    qitems.clear();

    // Also try empty path
    items.push_back(qitems);
    
    for (std::vector<QPath>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a QPath"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, Transform3DPath) {
    std::vector<Transform3DPath> items;
    std::vector<Transform3D<> > titems;

    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.128173, -0.297606, 0.946046), Rotation3D<>(0.3317661, 0.9381616, 0.09891459, -0.5708159, 0.2831207, -0.7707217, -0.7510662, 0.1992373, 0.6294474)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.599348, 0.144885, 0.787268), Rotation3D<>(0.7760383, 0.4833419, 0.4051483, -0.1534619, 0.7678005, -0.6220385, -0.6117303, 0.4205509, 0.6700172)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.0373863, 0.206556, -0.97772), Rotation3D<>(-0.003399149, -0.8786336, -0.4774845, 0.9403121, -0.1652964, 0.297473, -0.3402963, -0.4479733, 0.8267517)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.841704, 0.528854, -0.108845), Rotation3D<>(0.02146604, 0.4787316, -0.8776988, 0.8233708, 0.4895051, 0.2871328, 0.5670976, -0.7288352, -0.3836659)));
    titems.push_back(Transform3D<>(Vector3D<>(0.25201, -0.0695093, -0.965225), Rotation3D<>(0.7473519, 0.3497489, -0.5649255, 0.6592374, -0.2842517, 0.6961372, 0.08289218, -0.8926795, -0.4430036)));
    titems.push_back(Transform3D<>(Vector3D<>(0.154929, 0.987479, 0.0296881), Rotation3D<>(0.9857721, 0.04051401, 0.1631317, -0.04776038, 0.9980275, 0.04074474, -0.1611592, -0.04795626, 0.9857626)));
    titems.push_back(Transform3D<>(Vector3D<>(0.228879, -0.842727, 0.487263), Rotation3D<>(-0.4167191, -0.898851, 0.1356912, 0.87112, -0.3522036, 0.3422024, -0.2597981, 0.2608056, 0.9297771)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(-0.15146, -0.636242, 0.756476), Rotation3D<>(0.93294987, 0.21697526, -0.28727387, -0.13285456, 0.94914826, 0.28542467, 0.33459559, -0.22812127, 0.9143339)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.431606, -0.898602, 0.0789323), Rotation3D<>(-0.49982241, 0.32544564, -0.80265976, -0.56393633, -0.82565545, 0.016398039, -0.65738374, 0.46084511, 0.59621171)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.747131, -0.58661, 0.312543), Rotation3D<>(-0.77790463, 0.16963775, -0.60505159, -0.066535663, 0.93522296, 0.34775137, 0.6248499, 0.31077491, -0.71622731)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.245954, -0.867831, 0.431713), Rotation3D<>(-0.45548892, -0.88430274, -0.10265719, -0.66323347, 0.41399864, -0.62347935, 0.59384444, -0.21590225, -0.77507097)));
    titems.push_back(Transform3D<>(Vector3D<>(0.358033, 0.553017, 0.752319), Rotation3D<>(0.68281284, 0.17846936, 0.70845982, -0.59366858, 0.70072349, 0.39565668, -0.42582184, -0.69074979, 0.58441465)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.202799, -0.561398, -0.802312), Rotation3D<>(-0.22803737, 0.74510213, 0.62675495, 0.17442213, -0.60204016, 0.77918199, 0.95790181, 0.28700254, 0.007325355)));
    titems.push_back(Transform3D<>(Vector3D<>(0.821768, -0.294349, -0.48791), Rotation3D<>(0.15385748, -0.91617989, -0.37005713, -0.98386199, -0.17667041, 0.028339799, -0.091342498, 0.35972485, -0.92857664)));
    titems.push_back(Transform3D<>(Vector3D<>(0.0947497, -0.968762, 0.229179), Rotation3D<>(-0.67001955, -0.74222023, -0.013524834, -0.42030239, 0.39430849, -0.81723113, 0.61189844, -0.54187632, -0.57615134)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(-0.234126, -0.938533, 0.253656), Rotation3D<>(-0.7699476, -0.241163, -0.5907801, -0.3729896, -0.5811066, 0.7233214, -0.5177445, 0.7772744, 0.3574703)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.252482, -0.967311, 0.0237235), Rotation3D<>(0.9620877, 0.253362, -0.1009705, -0.2086533, 0.4453125, -0.8707242, -0.1756449, 0.8587808, 0.4812945)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.779311, 0.626539, 0.0110964), Rotation3D<>(-0.4855858, -0.8682503, 0.1017243, -0.1406307, 0.1924341, 0.9711808, -0.8628032, 0.457286, -0.215546)));
    titems.push_back(Transform3D<>(Vector3D<>(0.39525, 0.366508, -0.842288), Rotation3D<>(0.3148548, 0.7963973, 0.5163504, 0.5853031, -0.59118, 0.5549113, 0.7471859, 0.127505, -0.6522697)));
    titems.push_back(Transform3D<>(Vector3D<>(0.0133588, 0.714808, -0.699193), Rotation3D<>(0.8938847, -0.439516, 0.08829378, 0.1104089, 0.4067255, 0.906854, -0.4344882, -0.8008745, 0.4120922)));
    titems.push_back(Transform3D<>(Vector3D<>(0.00795702, -0.999457, -0.0319707), Rotation3D<>(0.6344802, 0.08954372, -0.7677348, -0.5432267, -0.6549324, -0.5253268, -0.5498541, 0.7503636, -0.3668991)));
    titems.push_back(Transform3D<>(Vector3D<>(0.660847, -0.640334, 0.391476), Rotation3D<>(0.3004718, 0.8670014, 0.3975238, 0.9537393, -0.2687901, -0.1346595, -0.00989952, 0.4195955, -0.9076572)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.452912, -0.138448, 0.88074), Rotation3D<>(-0.6115657, 0.7761471, 0.1535677, -0.7856076, -0.6187233, -0.001500034, 0.09385167, -0.1215613, 0.988137)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.560012, 0.3037, -0.770813), Rotation3D<>(-0.4614149, 0.8240499, 0.3286913, -0.2137088, -0.4628167, 0.8603077, 0.8610603, 0.3267146, 0.3896572)));
    titems.push_back(Transform3D<>(Vector3D<>(0.393007, -0.919501, 0.00795375), Rotation3D<>(0.5668984, -0.04821849, -0.8223753, -0.2284423, 0.9499324, -0.2131726, 0.7914798, 0.3087125, 0.5275)));
    titems.push_back(Transform3D<>(Vector3D<>(0.241934, 0.810525, -0.533402), Rotation3D<>(0.4749964, 0.8781155, 0.05737278, 0.830169, -0.4255265, -0.3602035, -0.2918866, 0.2187245, -0.9311078)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.381532, -0.340843, -0.85922), Rotation3D<>(-0.1824416, -0.9314975, 0.3146862, -0.8107753, -0.03852444, -0.5840884, 0.5562, -0.3617019, -0.7482067)));
    titems.push_back(Transform3D<>(Vector3D<>(0.349149, 0.253494, -0.902128), Rotation3D<>(-0.8328868, 0.5525161, -0.0320247, -0.2710974, -0.4577427, -0.8467454, -0.4824995, -0.6965612, 0.5310336)));
    titems.push_back(Transform3D<>(Vector3D<>(0.847659, -0.485661, -0.213557), Rotation3D<>(0.6909343, 0.7199057, 0.06592143, 0.6534732, -0.6609578, 0.3689275, 0.3091643, -0.2118268, -0.9271175)));
    titems.push_back(Transform3D<>(Vector3D<>(0.83263, -0.512919, -0.208908), Rotation3D<>(-0.06111007, 0.05237271, 0.9967561, -0.5664258, 0.8204289, -0.0778349, -0.8218439, -0.5693449, -0.02047121)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.672228, 0.347075, 0.653949), Rotation3D<>(-0.2793909, 0.75009029, 0.59942079, 0.23823365, -0.55060706, 0.80004787, 0.93015346, 0.3663283, -0.024862146)));
    titems.push_back(Transform3D<>(Vector3D<>(0.131866, -0.614153, 0.778092), Rotation3D<>(-0.86655317, -0.2019111, 0.45641813, -0.28541823, -0.54971672, -0.78507832, 0.40941671, -0.81058216, 0.41872966)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.651956, -0.300922, -0.695989), Rotation3D<>(-0.50373103, -0.48787172, -0.71290689, 0.36421346, -0.86826616, 0.33684186, -0.78332854, -0.089972589, 0.61506205)));
    titems.push_back(Transform3D<>(Vector3D<>(0.30307, -0.728787, 0.614019), Rotation3D<>(-0.57108981, -0.62032119, -0.53764119, 0.64300468, 0.069103091, -0.76273832, 0.51029541, -0.78129789, 0.35940536)));
    
    items.push_back(titems);
    titems.clear();

    // Also try empty path
    items.push_back(titems);

    for (std::vector<Transform3DPath>::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Transform3DPath"), *it));

        testSaverAndLoader(map);
    }
}

TEST(DOMPropertyMapSaveAndLoad, StringList) {
    rw::common::PropertyMap map;
    std::vector<std::string> list;
    list.push_back(std::string("first string"));
    list.push_back(std::string("second string"));
    list.push_back(std::string("third string"));
    EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a StringList"), list));

    testSaverAndLoader(map);
}

TEST(DOMPropertyMapSaveAndLoad, IntList) {
    rw::common::PropertyMap map;
    std::vector<int> list;
    list.push_back(920501);
    list.push_back(518590);
    list.push_back(133013);
    EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of an IntList"), list));

    testSaverAndLoader(map);
}

TEST(DOMPropertyMapSaveAndLoad, DoubleList) {
    rw::common::PropertyMap map;
    std::vector<double> list;
    list.push_back(3.5);
    list.push_back(0.45);
    list.push_back(3.1);
    EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a DoubleList"), list));

    testSaverAndLoader(map);
}
