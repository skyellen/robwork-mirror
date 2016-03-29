#include <gtest/gtest.h>

#include <rw/loaders/dom/DOMPropertyMapSaver.hpp>
#include <rw/loaders/dom/DOMPropertyMapLoader.hpp>

#include <rw/trajectory/Path.hpp>

#include <rw/math/Math.hpp>

#include <algorithm>
#include <sstream>

// Currently enabling these cause the generated binary to cause a memory error / segmentation fault
#define ENABLE_TRANSFORM3DPATH 1
#define ENABLE_TRANSFORM3D 1
#define ENABLE_ROTATION3D 1

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
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Vector2D: {
            const rw::common::Property<Vector2D<> >* pa = rw::common::toProperty<Vector2D<> >(a);
            const rw::common::Property<Vector2D<> >* pb = rw::common::toProperty<Vector2D<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Q: {
            const rw::common::Property<Q>* pa = rw::common::toProperty<Q>(a);
            const rw::common::Property<Q>* pb = rw::common::toProperty<Q>(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Transform3D: {
            const rw::common::Property<Transform3D<> >* pa = rw::common::toProperty<Transform3D<> >(a);
            const rw::common::Property<Transform3D<> >* pb = rw::common::toProperty<Transform3D<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Rotation3D: {
            const rw::common::Property<Rotation3D<> >* pa = rw::common::toProperty<Rotation3D<> >(a);
            const rw::common::Property<Rotation3D<> >* pb = rw::common::toProperty<Rotation3D<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::RPY: {
            const rw::common::Property<RPY<> >* pa = rw::common::toProperty<RPY<> >(a);
            const rw::common::Property<RPY<> >* pb = rw::common::toProperty<RPY<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::EAA: {
            const rw::common::Property<EAA<> >* pa = rw::common::toProperty<EAA<> >(a);
            const rw::common::Property<EAA<> >* pb = rw::common::toProperty<EAA<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Quaternion: {
            const rw::common::Property<Quaternion<> >* pa = rw::common::toProperty<Quaternion<> >(a);
            const rw::common::Property<Quaternion<> >* pb = rw::common::toProperty<Quaternion<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::Rotation2D: {
            const rw::common::Property<Rotation2D<> >* pa = rw::common::toProperty<Rotation2D<> >(a);
            const rw::common::Property<Rotation2D<> >* pb = rw::common::toProperty<Rotation2D<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
            break;
        }
        case rw::common::PropertyType::VelocityScrew6D: {
            const rw::common::Property<VelocityScrew6D<> >* pa = rw::common::toProperty<VelocityScrew6D<> >(a);
            const rw::common::Property<VelocityScrew6D<> >* pb = rw::common::toProperty<VelocityScrew6D<> >(b);
            EXPECT_EQ(pa->getValue(), pb->getValue());
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
                    EXPECT_EQ(pa->getValue().at(index), pb->getValue().at(index));
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
                    EXPECT_EQ(pa->getValue().at(index), pb->getValue().at(index));
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
        EXPECT_TRUE(item.add(std::string("double"), std::string("The description of a Double"), 0.4d));

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
    items.push_back(-0.1823d);
    items.push_back(-0.0d);
    items.push_back(0.0d);
    items.push_back(-1.86d);
    items.push_back(1.982d);
    items.push_back(-1235.2d);
    items.push_back(7823.9332d);
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
#if ENABLE_TRANSFORM3D
TEST(DOMPropertyMapSaveAndLoad, Transform3D) {
    std::vector<Transform3D<> > items;

    items.push_back(Transform3D<>(Vector3D<>(0.296485, -0.950844, -0.0893953), Rotation3D<>(-0.550979, -0.833006, -0.0502317, -0.823557, 0.55248, -0.128527, 0.134816, -0.0294471, -0.990433)));
    items.push_back(Transform3D<>(Vector3D<>(0.347301, 0.809361, -0.47362), Rotation3D<>(0.682813, 0.178469, 0.70846, -0.593669, 0.700723, 0.395657, -0.425822, -0.69075, 0.584415)));
    items.push_back(Transform3D<>(Vector3D<>(-0.998239, -0.0229768, 0.0546927), Rotation3D<>(-0.116715, -0.771598, -0.625311, -0.438901, -0.524726, 0.729403, -0.890923, 0.359582, -0.277412)));
    items.push_back(Transform3D<>(Vector3D<>(-0.167873, 0.902236, -0.397227), Rotation3D<>(-0.769948, -0.241163, -0.59078, -0.37299, -0.581107, 0.723321, -0.517745, 0.777274, 0.35747)));
    items.push_back(Transform3D<>(Vector3D<>(-0.0504132, -0.856888, -0.513031), Rotation3D<>(-0.0272354, 0.191063, -0.9812, 0.881021, -0.459169, -0.113866, -0.472292, -0.867559, -0.155825)));
    items.push_back(Transform3D<>(Vector3D<>(0.182764, 0.467202, 0.865054), Rotation3D<>(0.893885, -0.439516, 0.0882938, 0.110409, 0.406725, 0.906854, -0.434488, -0.800875, 0.412092)));
    items.push_back(Transform3D<>(Vector3D<>(-0.392463, 0.883221, 0.256696), Rotation3D<>(-0.143229, 0.810344, 0.568179, 0.788136, 0.440629, -0.429754, -0.598605, 0.386249, -0.701772)));
    items.push_back(Transform3D<>(Vector3D<>(-0.118084, 0.44222, -0.889099), Rotation3D<>(-0.461415, 0.82405, 0.328691, -0.213709, -0.462817, 0.860308, 0.86106, 0.326715, 0.389657)));
    items.push_back(Transform3D<>(Vector3D<>(0.0857885, -0.990521, -0.10728), Rotation3D<>(-0.249034, -0.897682, -0.363524, 0.429445, 0.23408, -0.872229, 0.868078, -0.373328, 0.327211)));
    for (std::vector<Transform3D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Transform3D"), *it));

        testSaverAndLoader(map);
    }
}
#endif
#if ENABLE_ROTATION3D
TEST(DOMPropertyMapSaveAndLoad, Rotation3D) {
    std::vector<Rotation3D<> > items;

    items.push_back(Rotation3D<>(0.331766, 0.938162, 0.0989146, -0.570816, 0.283121, -0.770722, -0.751066, 0.199237, 0.629447));
    items.push_back(Rotation3D<>(0.776038, 0.483342, 0.405148, -0.153462, 0.7678, -0.622038, -0.61173, 0.420551, 0.670017));
    items.push_back(Rotation3D<>(-0.00339915, -0.878634, -0.477484, 0.940312, -0.165296, 0.297473, -0.340296, -0.447973, 0.826752));
    items.push_back(Rotation3D<>(0.021466, 0.478732, -0.877699, 0.823371, 0.489505, 0.287133, 0.567098, -0.728835, -0.383666));
    items.push_back(Rotation3D<>(0.747352, 0.349749, -0.564926, 0.659237, -0.284252, 0.696137, 0.0828922, -0.89268, -0.443004));
    items.push_back(Rotation3D<>(0.985772, 0.040514, 0.163132, -0.0477604, 0.998027, 0.0407447, -0.161159, -0.0479563, 0.985763));
    items.push_back(Rotation3D<>(-0.416719, -0.898851, 0.135691, 0.87112, -0.352204, 0.342202, -0.259798, 0.260806, 0.929777));
    items.push_back(Rotation3D<>(0.449831, 0.0716641, 0.890234, -0.269712, 0.961137, 0.0589122, -0.851415, -0.266607, 0.451678));
    items.push_back(Rotation3D<>(0.93295, 0.216975, -0.287274, -0.132855, 0.949148, 0.285425, 0.334596, -0.228121, 0.914334));
    for (std::vector<Rotation3D<> >::const_iterator it = items.begin(); it != items.end(); ++it) {
        rw::common::PropertyMap map;
        EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a Rotation3D"), *it));

        testSaverAndLoader(map);
    }
}
#endif

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

#if ENABLE_TRANSFORM3DPATH
TEST(DOMPropertyMapSaveAndLoad, Transform3DPath) {
    std::vector<Transform3DPath> items;
    std::vector<Transform3D<> > titems;

    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.128173, -0.297606, 0.946046), Rotation3D<>(-0.74816, -0.220389, 0.625848, -0.445398, 0.865945, -0.227507, -0.49181, -0.448963, -0.746026)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.599348, 0.144885, 0.787268), Rotation3D<>(0.021466, 0.478732, -0.877699, 0.823371, 0.489505, 0.287133, 0.567098, -0.728835, -0.383666)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.0373863, 0.206556, -0.97772), Rotation3D<>(0.823355, 0.407011, 0.395511, -0.410789, 0.908257, -0.0795062, -0.391585, -0.0970095, 0.915014)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.841704, 0.528854, -0.108845), Rotation3D<>(0.449831, 0.0716641, 0.890234, -0.269712, 0.961137, 0.0589122, -0.851415, -0.266607, 0.451678)));
    titems.push_back(Transform3D<>(Vector3D<>(0.25201, -0.0695093, -0.965225), Rotation3D<>(-0.00270293, -0.89361, 0.448835, 0.67105, -0.334393, -0.66172, 0.741407, 0.299402, 0.600561)));
    titems.push_back(Transform3D<>(Vector3D<>(0.154929, 0.987479, 0.0296881), Rotation3D<>(-0.455489, -0.884303, -0.102657, -0.663233, 0.413999, -0.623479, 0.593844, -0.215902, -0.775071)));
    titems.push_back(Transform3D<>(Vector3D<>(0.228879, -0.842727, 0.487263), Rotation3D<>(0.874495, -0.479529, 0.072871, 0.0894627, 0.307127, 0.947454, -0.476712, -0.822025, 0.311481)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(-0.15146, -0.636242, 0.756476), Rotation3D<>(-0.67002, -0.74222, -0.0135248, -0.420302, 0.394308, -0.817231, 0.611898, -0.541876, -0.576151)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.431606, -0.898602, 0.0789323), Rotation3D<>(-0.0844332, 0.64567, 0.758934, -0.805904, 0.403674, -0.433088, -0.585994, -0.648195, 0.486265)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.747131, -0.58661, 0.312543), Rotation3D<>(0.314855, 0.796397, 0.51635, 0.585303, -0.59118, 0.554911, 0.747186, 0.127505, -0.65227)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.245954, -0.867831, 0.431713), Rotation3D<>(0.254963, -0.874508, 0.412589, -0.570891, 0.208245, 0.794177, -0.780434, -0.438029, -0.446154)));
    titems.push_back(Transform3D<>(Vector3D<>(0.358033, 0.553017, 0.752319), Rotation3D<>(-0.611566, 0.776147, 0.153568, -0.785608, -0.618723, -0.00150003, 0.0938517, -0.121561, 0.988137)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.202799, -0.561398, -0.802312), Rotation3D<>(0.813152, -0.404463, -0.418561, 0.392717, 0.912012, -0.11835, 0.429602, -0.0681394, 0.900444)));
    titems.push_back(Transform3D<>(Vector3D<>(0.821768, -0.294349, -0.48791), Rotation3D<>(-0.182442, -0.931498, 0.314686, -0.810775, -0.0385244, -0.584088, 0.5562, -0.361702, -0.748207)));
    titems.push_back(Transform3D<>(Vector3D<>(0.0947497, -0.968762, 0.229179), Rotation3D<>(-0.149109, -0.717196, 0.680732, -0.765136, 0.519767, 0.380011, -0.626365, -0.46419, -0.626255)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(-0.234126, -0.938533, 0.253656), Rotation3D<>(-0.279391, 0.75009, 0.599421, 0.238234, -0.550607, 0.800048, 0.930153, 0.366328, -0.0248621)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.252482, -0.967311, 0.0237235), Rotation3D<>(0.439868, 0.168623, 0.88209, 0.595296, -0.790167, -0.145804, 0.672412, 0.589239, -0.44795)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.779311, 0.626539, 0.0110964), Rotation3D<>(-0.0360424, -0.433045, -0.900652, 0.948205, -0.299441, 0.10603, -0.315608, -0.850181, 0.421408)));
    titems.push_back(Transform3D<>(Vector3D<>(0.39525, 0.366508, -0.842288), Rotation3D<>(-0.428594, 0.902566, 0.0410046, -0.838179, -0.380256, -0.390974, -0.337288, -0.201938, 0.919488)));
    titems.push_back(Transform3D<>(Vector3D<>(0.0133588, 0.714808, -0.699193), Rotation3D<>(0.969631, -0.117018, 0.214761, 0.241006, 0.60657, -0.757621, -0.0416124, 0.786371, 0.616351)));
    titems.push_back(Transform3D<>(Vector3D<>(0.00795702, -0.999457, -0.0319707), Rotation3D<>(-0.766306, -0.639984, 0.0565314, -0.0357374, -0.0453937, -0.99833, 0.641481, -0.767047, 0.0119141)));
    titems.push_back(Transform3D<>(Vector3D<>(0.660847, -0.640334, 0.391476), Rotation3D<>(-0.280558, -0.642372, -0.713193, 0.0776079, 0.725421, -0.683916, 0.956694, -0.247228, -0.15367)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.452912, -0.138448, 0.88074), Rotation3D<>(0.675763, 0.224011, 0.702256, 0.19588, -0.973023, 0.121891, 0.710616, 0.0551886, -0.701412)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.560012, 0.3037, -0.770813), Rotation3D<>(-0.49229, 0.849918, 0.18786, -0.851152, -0.515213, 0.100475, 0.182184, -0.110434, 0.977043)));
    titems.push_back(Transform3D<>(Vector3D<>(0.393007, -0.919501, 0.00795375), Rotation3D<>(-0.222513, 0.91203, 0.344513, -0.855417, -0.352164, 0.379792, 0.467707, -0.210194, 0.858527)));
    titems.push_back(Transform3D<>(Vector3D<>(0.241934, 0.810525, -0.533402), Rotation3D<>(0.868004, 0.45241, -0.204683, -0.456537, 0.564967, -0.687303, -0.195303, 0.690027, 0.696936)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.381532, -0.340843, -0.85922), Rotation3D<>(-0.613499, 0.785703, 0.0793119, 0.187979, 0.242845, -0.951677, -0.766996, -0.568944, -0.296681)));
    titems.push_back(Transform3D<>(Vector3D<>(0.349149, 0.253494, -0.902128), Rotation3D<>(-0.909548, 0.109696, 0.40086, 0.364712, -0.251789, 0.89643, 0.199267, 0.961545, 0.189007)));
    titems.push_back(Transform3D<>(Vector3D<>(0.847659, -0.485661, -0.213557), Rotation3D<>(-0.808799, -0.201601, -0.55245, -0.272542, -0.703939, 0.65589, -0.521119, 0.681049, 0.5144)));
    titems.push_back(Transform3D<>(Vector3D<>(0.83263, -0.512919, -0.208908), Rotation3D<>(-0.325428, 0.525985, 0.785772, 0.666313, 0.71719, -0.204123, -0.670913, 0.457143, -0.583864)));

    items.push_back(titems);
    titems.clear();

    titems.push_back(Transform3D<>(Vector3D<>(0.672228, 0.347075, 0.653949), Rotation3D<>(-0.680129, 0.621592, 0.388648, 0.241143, -0.310952, 0.919325, 0.692296, 0.71898, 0.0615951)));
    titems.push_back(Transform3D<>(Vector3D<>(0.131866, -0.614153, 0.778092), Rotation3D<>(-0.346729, -0.86472, 0.36337, 0.934655, -0.351046, 0.0564578, 0.0787393, 0.359201, 0.929933)));
    titems.push_back(Transform3D<>(Vector3D<>(-0.651956, -0.300922, -0.695989), Rotation3D<>(0.941499, 0.292745, -0.166971, 0.271587, -0.952407, -0.138428, -0.199549, 0.0849824, -0.976196)));
    titems.push_back(Transform3D<>(Vector3D<>(0.30307, -0.728787, 0.614019), Rotation3D<>(0.705911, 0.628831, -0.325977, -0.70543, 0.582778, -0.403409, -0.063704, 0.514724, 0.854986)));
    
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
#endif

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
    list.push_back(3.5d);
    list.push_back(0.45d);
    list.push_back(3.1d);
    EXPECT_TRUE(map.add(std::string("identifier"), std::string("The description of a DoubleList"), list));

    testSaverAndLoader(map);
}
