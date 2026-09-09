#include "cell_display.hpp"
#include <algorithm>
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <limits>
#include <set>
#include <tuple>
#include <QApplication>
#include <QEventLoop>
#include <QTimer>
#include <QTemporaryFile>
#include <QTemporaryDir>
#include <OgreBillboardChain.h>
#include <OgreMaterial.h>
#include <OgrePass.h>
#include <OgreSceneNode.h>
#include <OgreTechnique.h>
#include <rclcpp/rclcpp.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/display_group.hpp>
#include <rviz_common/visualization_frame.hpp>
#include <rviz_common/visualization_manager.hpp>
#include <rviz_common/ros_integration/ros_node_abstraction.hpp>
#include <interfaces/msg/cell_geometry.hpp>
#include <interfaces/msg/cell_colors.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>
using Geometry = interfaces::msg::CellGeometry;
using Colors = interfaces::msg::CellColors;
using Marker = visualization_msgs::msg::Marker;
using Array = visualization_msgs::msg::MarkerArray;
using Vertex = std::tuple<float,float,float,float,float,float,float,float,int,int,bool>;
void pump(int ms=160) { QEventLoop loop; QTimer::singleShot(ms,&loop,&QEventLoop::quit); loop.exec(); }
std::vector<Vertex> vertices(Ogre::SceneNode * node) {
    std::vector<Vertex> result;
    for (auto obj : node->getAttachedObjects()) {
        auto chain=dynamic_cast<Ogre::BillboardChain *>(obj);
        if (!chain || !chain->getVisible()) continue;
        auto pass=chain->getMaterial()->getTechnique(0)->getPass(0);
        for (size_t i=0;i<chain->getNumberOfChains();++i) for(size_t j=0;j<chain->getNumChainElements(i);++j) {
            auto e=chain->getChainElement(i,j);
            auto p=node->_getFullTransform()*e.position;
            result.emplace_back(p.x,p.y,p.z,e.colour.r,e.colour.g,e.colour.b,e.colour.a,e.width,
                pass->getSourceBlendFactor(),pass->getDestBlendFactor(),pass->getDepthWriteEnabled());
        }
    }
    for(auto child:node->getChildren()) {
        auto sub=vertices(static_cast<Ogre::SceneNode *>(child)); result.insert(result.end(),sub.begin(),sub.end());
    }
    std::sort(result.begin(),result.end()); return result;
}
std::set<std::string> objects(Ogre::SceneNode * node) {
    std::set<std::string> out;
    for(auto obj:node->getAttachedObjects()) out.insert(obj->getName());
    for(auto child:node->getChildren()) { auto sub=objects(static_cast<Ogre::SceneNode *>(child)); out.insert(sub.begin(),sub.end()); }
    return out;
}
std_msgs::msg::ColorRGBA color(float r,float g,float b,float a) {
    std_msgs::msg::ColorRGBA c; c.r=r;c.g=g;c.b=b;c.a=a;return c;
}
Geometry geometry(std::string id="map-A",size_t count=3) {
    Geometry g;g.header.frame_id="map";g.geometry_id=id;g.offsets.push_back(0);
    for(size_t c=0;c<count;++c) {
        for(auto xy:{std::pair<double,double>{0,0},{1,0},{1,1},{0,1}}) {
            geometry_msgs::msg::Point p;p.x=(c%1000)*2+xy.first;p.y=(c/1000)*2+xy.second;p.z=c%3;
            g.points.push_back(p);
        }
        g.offsets.push_back(g.points.size());
    }
    return g;
}
Array legacy(const Geometry & g,const Colors & c) {
    Array out;Marker clear;clear.action=Marker::DELETEALL;out.markers.push_back(clear);
    std::vector<bool> order;
    for(auto col:c.colors) if(std::find(order.begin(),order.end(),col.a>=.9998f)==order.end()) order.push_back(col.a>=.9998f);
    for(bool opaque:order) {
        Marker m;m.header=c.header;m.ns="legacy";m.id=opaque;m.type=Marker::LINE_LIST;m.pose.orientation.w=1;m.scale.x=.08;
        bool first=true;
        for(size_t cell=0;cell<c.colors.size();++cell) if((c.colors[cell].a>=.9998f)==opaque) {
            if(first){m.color=c.colors[cell];first=false;}
            for(size_t i=g.offsets[cell];i<g.offsets[cell+1];++i) {
                m.points.push_back(g.points[i]);m.points.push_back(g.points[i+1<g.offsets[cell+1]?i+1:g.offsets[cell]]);
                m.colors.push_back(c.colors[cell]);m.colors.push_back(c.colors[cell]);
            }
        }
        out.markers.push_back(m);
    }
    return out;
}
void checkWireContract() {
    using namespace visualization;
    auto valid=geometry();
    assert(cellGeometryError(valid).empty());
    auto invalid=valid; invalid.offsets={0,5,4,12};
    assert(!cellGeometryError(invalid).empty());
    invalid=valid; ++invalid.offsets.back();
    assert(!cellGeometryError(invalid).empty());
    invalid=valid; invalid.points[0].z=std::numeric_limits<double>::quiet_NaN();
    assert(!cellGeometryError(invalid).empty());
    invalid=valid; invalid.header.frame_id="base_link";
    assert(!cellGeometryError(invalid).empty());
    Colors colors; colors.header.frame_id="map"; colors.geometry_id=valid.geometry_id;
    colors.colors={color(.1,.2,.3,.4)};
    assert(cellColorsError(colors).empty());
    colors.colors[0].a=std::numeric_limits<float>::infinity();
    assert(!cellColorsError(colors).empty());
    colors.header.frame_id="invalid"; colors.geometry_id=""; colors.colors.clear();
    assert(cellColorsError(colors).empty());
    for(float alpha:{std::nextafter(.9998f,0.f),.9998f,std::nextafter(.9998f,1.f)})
        assert(cellIsOpaque(color(1,0,0,alpha))==(double(alpha)>=.9998));
}
int main(int argc,char**argv) {
    checkWireContract();
    if(argc>1 && std::string(argv[1])=="--contract") return 0;
    if (!std::getenv("DISPLAY")) return 77;
    QTemporaryDir logs; assert(logs.isValid());
    setenv("ROS_DOMAIN_ID", "216", 1);
    setenv("ROS_LOCALHOST_ONLY", "1", 1);
    setenv("ROS_LOG_DIR", logs.path().toUtf8().constData(), 1);
    rclcpp::init(argc,argv);QApplication app(argc,argv);
    QTemporaryFile config;
    assert(config.open());
    config.write("Panels: []\nVisualization Manager:\n  Class: ''\n  Global Options:\n    Fixed Frame: map\n    Frame Rate: 30\n  Displays: []\n");
    config.flush();
    auto ros=std::make_shared<rviz_common::ros_integration::RosNodeAbstraction>("cell_colors_integration");
    rviz_common::VisualizationFrame frame(ros);frame.setApp(&app);frame.setSplashPath("");
    frame.initialize(ros,config.fileName());
    auto & manager=*frame.getManager();manager.setFixedFrame("map");frame.resize(480,320);frame.show();
    auto make=[&](const char * cls,const char * name,const char * topic){auto d=manager.createDisplay(cls,name,true);assert(d);d->setTopic(topic,"");return d;};
    auto compact=make("visualization/CellColors","compact","/cell_check/colors");
    auto native=make("rviz_default_plugins/MarkerArray","native","/cell_check/native");
    auto node=ros->get_raw_node();
    auto gp=node->create_publisher<Geometry>("/visualization/cell_geometry",rclcpp::QoS(1).reliable().transient_local());
    auto cp=node->create_publisher<Colors>("/cell_check/colors",rclcpp::QoS(1));
    auto np=node->create_publisher<Array>("/cell_check/native",rclcpp::QoS(1));
    auto tfp=node->create_publisher<tf2_msgs::msg::TFMessage>("/tf",rclcpp::QoS(100));
    pump(800);
    auto g=geometry();Colors c;c.header.frame_id="map";c.geometry_id=g.geometry_id;
    c.colors={color(1,.1,.1,.2),color(.5,.5,.5,.6),color(1,.1,.1,1)};
    cp->publish(c);pump();assert(vertices(compact->getSceneNode()).empty());
    gp->publish(g);np->publish(legacy(g,c));pump(500);
    auto equal=[&](){auto a=vertices(compact->getSceneNode()),b=vertices(native->getSceneNode());
        if(a!=b){std::cerr<<"Mismatch vertices compact="<<a.size()<<" native="<<b.size()<<std::endl;}
        assert(a==b&&!a.empty());};
    equal();auto initial=objects(compact->getSceneNode());
    auto send=[&](){cp->publish(c);np->publish(legacy(g,c));pump();equal();};
    c.header.stamp.sec=12;c.colors[0]=color(.1,.7,.3,.8);send();assert(objects(compact->getSceneNode())==initial);
    send();assert(objects(compact->getSceneNode())==initial);
    c.colors[0].a=1;c.colors[2].a=.4;send();
    c.colors[0].a=.9998f;send();c.colors[0].a=std::nextafter(.9998f,0.f);send();
    Colors empty=c;empty.colors.clear();cp->publish(empty);pump();assert(vertices(compact->getSceneNode()).empty());send();
    auto bad=c;bad.colors.pop_back();cp->publish(bad);pump();assert(vertices(compact->getSceneNode()).empty());send();
    bad=c;bad.colors[0].r=std::numeric_limits<float>::quiet_NaN();cp->publish(bad);pump();assert(vertices(compact->getSceneNode()).empty());send();
    bad=c;bad.geometry_id="map-B";cp->publish(bad);pump();equal();
    g.geometry_id="map-B";gp->publish(g);c.geometry_id="map-B";np->publish(legacy(g,c));pump();equal();
    compact->reset();pump();assert(vertices(compact->getSceneNode()).empty());send();
    compact->setEnabled(false);assert(vertices(compact->getSceneNode()).empty());compact->setEnabled(true);pump();send();
    auto late=make("visualization/CellColors","late","/cell_check/colors");pump(500);send();
    assert(vertices(late->getSceneNode())==vertices(compact->getSceneNode()));late->setEnabled(false);
    // A clear must discard colors waiting for a new geometry.
    bad=c;bad.geometry_id="map-C";cp->publish(bad);pump();cp->publish(empty);pump();g.geometry_id="map-C";gp->publish(g);pump();
    assert(vertices(compact->getSceneNode()).empty());c.geometry_id=g.geometry_id;send();
    // Transform lookup uses each color message's source time, including time reversal.
    manager.setFixedFrame("fixed");pump();tf2_msgs::msg::TFMessage tf;
    for(int sec:{12,18}){geometry_msgs::msg::TransformStamped t;t.header.frame_id="fixed";t.child_frame_id="map";t.header.stamp.sec=sec;t.transform.translation.x=sec;t.transform.rotation.w=1;tf.transforms.push_back(t);}
    tfp->publish(tf);pump(500);c.header.stamp.sec=12;send();auto at12=vertices(compact->getSceneNode());c.header.stamp.sec=18;send();auto at18=vertices(compact->getSceneNode());
    assert(std::get<0>(at18.front())-std::get<0>(at12.front())==6);c.header.stamp.sec=12;send();
    c.header.stamp.sec=30;cp->publish(c);np->publish(legacy(g,c));pump();assert(vertices(compact->getSceneNode()).empty()&&vertices(native->getSceneNode()).empty());
    c.header.stamp.sec=18;send();
    // A topic change clears old colors while preserving the shared geometry cache.
    compact->setTopic("/cell_check/other", ""); pump();
    assert(vertices(compact->getSceneNode()).empty());
    auto other=node->create_publisher<Colors>("/cell_check/other",rclcpp::QoS(1));pump(500);
    other->publish(c);pump();equal();compact->setTopic("/cell_check/colors", "");pump();send();
    // Cross the native 16,384-endpoint chunk boundary and recolor every cell.
    manager.setFixedFrame("map"); pump();
    g=geometry("large-map",5000); c.geometry_id=g.geometry_id; c.colors.resize(5000);
    for(size_t i=0;i<c.colors.size();++i) c.colors[i]=color(float(i%17)/17,.1,.3,.6);
    gp->publish(g);pump();send();initial=objects(compact->getSceneNode());
    for(size_t i=0;i<c.colors.size();++i) c.colors[i]=color(.7,float(i%19)/19,.2,.4);
    send();assert(objects(compact->getSceneNode())==initial);
    for(size_t i=0;i<c.colors.size();i+=3) c.colors[i].a=1;
    send();
    manager.stopUpdate();std::cout<<"PASS: native XYZ/RGBA/width/blending/depth; color-only object reuse across chunks; opacity crossing; clear/invalid; pending/replacement; reset/late subscriber; exact-source TF/time reversal"<<std::endl;
}
