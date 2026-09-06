#include <osg/Camera>
#include <osg/GraphicsContext>
#include <osg/Geode>
#include <osg/Geometry>
#include <osg/Texture2D>
#include <osg/TriangleFunctor>
#include <osgDB/ReadFile>
#include <osgDB/WriteFile>
#include <osgViewer/Viewer>

#include <cstdlib>
#include <fstream>
#include <iomanip>
#include <iostream>

struct TriangleWriter {
    std::ostream* output;
    osg::Matrixd transform;

    void operator()(const osg::Vec3& first, const osg::Vec3& second, const osg::Vec3& third) {
        for (const auto& vertex : {first, second, third}) {
            const auto world = osg::Vec3d(vertex) * transform;
            *output << world.x() << ' ' << world.y() << ' ' << world.z() << ' ';
        }
        *output << '\n';
    }
};

struct WhiteMeshVisitor : osg::NodeVisitor {
    explicit WhiteMeshVisitor(std::ostream& destination)
        : osg::NodeVisitor(TRAVERSE_ALL_CHILDREN), output(destination) {}

    void apply(osg::Geode& geode) override {
        for (unsigned int index = 0; index < geode.getNumDrawables(); ++index) {
            auto* geometry = geode.getDrawable(index)->asGeometry();
            if (!geometry || !geometry->getStateSet()) {
                continue;
            }
            auto* texture = dynamic_cast<osg::Texture2D*>(geometry->getStateSet()->getTextureAttribute(0, osg::StateAttribute::TEXTURE));
            if (!texture || (texture->getName() != "Tx_Rm_White_01.rgb" && texture->getName() != "roadmark_White.rgb")) {
                continue;
            }
            osg::TriangleFunctor<TriangleWriter> writer;
            writer.output = &output;
            writer.transform = osg::computeLocalToWorld(getNodePath());
            geometry->accept(writer);
        }
    }

    std::ostream& output;
};

int main(int argc, char** argv) {
    if (argc != 7 && argc != 8 && argc != 4) {
        std::cerr << "render_map input.osgb output.png xmin ymin xmax ymax [pixels]\n"
            << "render_map input.osgb --white-mesh output.txt\n";
        return 1;
    }
    const auto scene = osgDB::readRefNodeFile(argv[1]);
    if (!scene) {
        return 2;
    }
    if (argc == 4) {
        if (std::string(argv[2]) != "--white-mesh") {
            return 1;
        }
        std::ofstream output(argv[3]);
        output << std::setprecision(12);
        WhiteMeshVisitor visitor(output);
        scene->accept(visitor);
        return output ? 0 : 4;
    }
    const double xmin = std::stod(argv[3]);
    const double ymin = std::stod(argv[4]);
    const double xmax = std::stod(argv[5]);
    const double ymax = std::stod(argv[6]);
    osg::ref_ptr<osg::GraphicsContext::Traits> traits = new osg::GraphicsContext::Traits;
    traits->readDISPLAY();
    traits->setUndefinedScreenDetailsToDefaultScreen();
    traits->width = argc == 8 ? std::stoi(argv[7]) : 1600;
    traits->height = traits->width;
    traits->pbuffer = true;
    traits->doubleBuffer = false;
    const auto context = osg::GraphicsContext::createGraphicsContext(traits);
    if (!context) {
        return 3;
    }
    osgViewer::Viewer viewer;
    viewer.setThreadingModel(osgViewer::Viewer::SingleThreaded);
    viewer.setSceneData(scene);
    auto camera = viewer.getCamera();
    camera->setGraphicsContext(context);
    camera->setViewport(0, 0, traits->width, traits->height);
    camera->setClearColor(osg::Vec4(0.15, 0.15, 0.15, 1.));
    camera->setComputeNearFarMode(osg::CullSettings::DO_NOT_COMPUTE_NEAR_FAR);
    // The aerial camera must use near road meshes; coarse LOD surfaces can cover road markings.
    camera->setLODScale(0.001f);
    camera->setProjectionMatrixAsOrtho(xmin, xmax, ymin, ymax, 1., 2000.);
    camera->setViewMatrixAsLookAt(osg::Vec3d(0., 0., 1000.), osg::Vec3d(0., 0., 0.), osg::Vec3d(0., 1., 0.));
    camera->setDrawBuffer(GL_FRONT);
    camera->setReadBuffer(GL_FRONT);
    osg::ref_ptr<osg::Image> image = new osg::Image;
    camera->attach(osg::Camera::COLOR_BUFFER, image);
    viewer.realize();
    for (int frame = 0; frame < 8; ++frame) {
        viewer.frame();
    }
    return osgDB::writeImageFile(*image, argv[2]) ? 0 : 4;
}
