#include "graspGenerationPlugin.h"

#include "QJsonObject.h"
#include <boost/foreach.hpp>
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>

#include <include/EGPlanner/egPlanner.h>
#include <include/EGPlanner/simAnnPlanner.h>
#include <include/EGPlanner/onLinePlanner.h>
#include <include/EGPlanner/guidedPlanner.h>
#include <include/EGPlanner/searchState.h>
#include <include/EGPlanner/searchEnergy.h>

#include <include/grasp.h>
#include <include/triangle.h>

#include <cmdline/cmdline.h>
#include "include/dbModelLoader.h"

#include <cstdlib>
#include <iostream>

#include "mongo/client/dbclient.h" // for the driver



using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;
using namespace mongo;


GraspGenerationPlugin::GraspGenerationPlugin() :
    mPlanner(NULL),
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false),
    num_steps(70000)
{

}

GraspGenerationPlugin::~GraspGenerationPlugin()
{
}


int GraspGenerationPlugin::init(int argc, char **argv)
{
    std::cout << "Starting GraspGenerationPlugin: " << std::endl ;
    std::cout << "Connecting to Mongo..." << std::endl ;

    mongo::client::GlobalInstance instance;
    if (!instance.initialized()) {
        std::cout << "failed to initialize the client driver: " << instance.status() << std::endl;
        return EXIT_FAILURE;
    }
    try {


        std::string uri = "mongodb://localhost:27017";
        std::string errmsg;

        ConnectionString cs = ConnectionString::parse(uri, errmsg);

        if (!cs.isValid()) {
            std::cout << "Error parsing connection string " << uri << ": " << errmsg << std::endl;
            return EXIT_FAILURE;
        }

        c = cs.connect(errmsg);
        if (!c) {
            std::cout << "couldn't connect : " << errmsg << std::endl;
            return EXIT_FAILURE;
        }





//        std::string errmsg;
//        ConnectionString cs = ConnectionString::parse("mongodb://localhost:27017", errmsg);
////        mongodb://tim:ilovetim@ds023418.mlab.com:23418/goparse

////        c.connect(cs.getServers().at(0));
////        c.auth(cs.getDatabase(), cs.getUser(), cs.getPassword());
////        c = cs.connect(errmsg);
////        c = static_cast<mongo::DBClientConnection*>(cs.connect(errmsg));
//        boost::scoped_ptr<DBClientBase> c(cs.connect(errmsg));
        std::cout << "connected ok to mongodb" << std::endl;
    } catch( const mongo::DBException &e ) {
        std::cout << "caught " << e.what() << std::endl;
    }


    std::cout << "Parsing Args..." << std::endl;
    cmdline::parser *parser = new cmdline::parser();

    parser->add<std::string>("mesh_filepath", 'c', "mesh_filepath",  false);
    parser->add<bool>("render", 'l', "render", false);

    parser->parse(argc, argv);

    if (parser->exist("render"))
    {
        render_it = parser->get<bool>("render");

    }
    else
    {
        render_it = false;
    }

    mesh_filepath = QString::fromStdString(parser->get<std::string>("mesh_filepath"));

    std::cout << "Args are: " << std::endl;
    std::cout << "render: " << render_it << "\n" ;
    std::cout << "mesh_filepath: " << mesh_filepath.toStdString().c_str() << "\n" ;

    std::cout << "Finished Init..." << std::endl;

    return 0;
}

//This loop is called over and over again. We do 3 different things
// 1) First step: start the planner
// 2) Middle steps: step the planner
// 3) Last step, save the grasps
int GraspGenerationPlugin::mainLoop()
{
    //start planner
    if (!plannerStarted)
    {
        startPlanner();
    }
    //let planner run.
    else if( (plannerStarted) && !plannerFinished )
    {
        stepPlanner();
    }
    //save grasps
    else if(plannerStarted && plannerFinished && (!evaluatingGrasps))
    {
        uploadResults();
    }

  return 0;
}

void GraspGenerationPlugin::startPlanner()
{
    std::cout << "Starting Planner\n" ;

    //TODO
    //here we need to get the hand and object from the cloud. rather than locally
    QString modelUrl = "http://borneo.cs.columbia.edu/modelnet/vision.cs.princeton.edu/projects/2014/ModelNet/data/pyramid/pyramid_000001463/pyramid_000001463.off";
    DbModelLoader loader;
//    loader.loadModelFromUrl(modelUrl, QString(""), QString("rubber"));

    modelJson = loader.loadRandomModel();

    std::cout << "FINISHED LOADING DEBUG" << std::endl;
//    graspItGUI->getMainWorld()->importBody("GraspableBody", mesh_filepath);
    //this is fine for now, in the future, we may change this
//    graspItGUI->getMainWorld()->importRobot("/home/timchunght/graspit/models/robots/pr2_gripper_2010/pr2_gripper_2010.xml");
//    graspItGUI->getMainWorld()->importRobot("/home/timchunght/graspit/models/robots/Barrett/Barrett.xml");

    mObject = graspItGUI->getMainWorld()->getGB(0);
    mObject->setMaterial(5);//rubber

    mHand = graspItGUI->getMainWorld()->getCurrentHand();
    mHand->getGrasp()->setObjectNoUpdate(mObject);
    mHand->getGrasp()->setGravity(false);

    mHandObjectState = new GraspPlanningState(mHand);
    mHandObjectState->setObject(mObject);
    mHandObjectState->setPositionType(SPACE_AXIS_ANGLE);
    mHandObjectState->setRefTran(mObject->getTran());
    mHandObjectState->reset();

    mPlanner = new SimAnnPlanner(mHand);
    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

//    mPlanner = new GuidedPlanner(mHand);
//    ((SimAnnPlanner*)mPlanner)->setModelState(mHandObjectState);

    mPlanner->setEnergyType(ENERGY_CONTACT_QUALITY);
    mPlanner->setContactType(CONTACT_PRESET);
    mPlanner->setMaxSteps(num_steps);

    mPlanner->resetPlanner();

    mPlanner->startPlanner();
    plannerStarted = true;
}

void GraspGenerationPlugin::stepPlanner()
{
    if ( mPlanner->getCurrentStep() >= num_steps)
    {
        mPlanner->stopPlanner();
        plannerFinished=true;
    }
}

void GraspGenerationPlugin::uploadResults()
{

    SearchEnergy *mEnergyCalculator = new SearchEnergy();
    mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
    mEnergyCalculator->setContactType(CONTACT_PRESET);

    int num_grasps = mPlanner->getListSize();
    std::cout << "Found " << num_grasps << " Grasps. " << std::endl;

    for(int i=0; i < num_grasps; i++)
    {
        GraspPlanningState gps = mPlanner->getGrasp(i);
        gps.execute(mHand);
        mHand->autoGrasp(render_it, 1.0, false);
        bool is_legal;
        double new_planned_energy;

        mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
        gps.setEnergy(new_planned_energy);
        gps.saveCurrentHandState();

        graspItGUI->getIVmgr()->getViewer()->render();
        //usleep(1000000);

        BSONObj p = toMongoGrasp(&gps, QString("ENERGY_CONTACT_QUALITY"));
        c->insert("test.grasps_dev", p);

    }
    // TODO: find a better way to die
    assert(false);
}

mongo::BSONObj GraspGenerationPlugin::toMongoGrasp(GraspPlanningState *gps, QString energyType)
{
    BSONObjBuilder grasp;
    BSONObjBuilder pose;
    BSONObjBuilder model;
    BSONObjBuilder energy;
    BSONArrayBuilder translation;
    BSONArrayBuilder rotation;
    BSONArrayBuilder dof;

    Hand *hand = gps->getHand();
    GraspableBody *body = gps->getObject();

    double dofVals [hand->getNumDOF()];
    hand->getDOFVals(dofVals);

    transf hand_pose = mHand->getPalm()->getTran();

    for(int dof_idx = 0; dof_idx < hand->getNumDOF(); dof_idx ++)
    {
        dof.append(dofVals[dof_idx]);
    }

    energy.append("type", "ENERGY_CONTACT_QUALITY");
    energy.append("value", gps->getEnergy());

    translation.append(hand_pose.translation().x()).append(hand_pose.translation().y()).append(hand_pose.translation().z());
    rotation.append(hand_pose.rotation().w).append(hand_pose.rotation().x).append(hand_pose.rotation().y).append(hand_pose.rotation().z);
    pose.append("translation", translation.arr());
    pose.append("rotation", rotation.arr());

    QString url = modelJson["url"].toString();
    QString modelName = modelJson["name"].toString();
    QString material = modelJson["material"].toString();
    double dimension = modelJson["dimension"].toDouble();

    model.append("name", modelName.toStdString());
    model.append("url", url.toStdString());
    model.append("material", material.toStdString());
    model.append("dimension", dimension);

    grasp.append("model", model.obj());
    grasp.append("hand", hand->getDBName().toStdString());
    grasp.append("energy", energy.obj());
    grasp.appendArray("dof", dof.arr());
    grasp.append("pose", pose.obj());

    return grasp.obj();
}

