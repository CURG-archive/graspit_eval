#include "graspGenerationPlugin.h"

#include "QJsonObject.h"
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


GraspEvalPlugin::GraspEvalPlugin() :
    mPlanner(NULL),
    plannerStarted(false),
    plannerFinished(false),
    evaluatingGrasps(false)

{

}

GraspEvalPlugin::~GraspEvalPlugin()
{
}        


int GraspEvalPlugin::init(int argc, char **argv)
{
    std::cout << "Starting GraspEvalPlugin: " << std::endl ;
    std::cout << "Connecting to Mongo..." << std::endl ;
    std::cout << "My Integer" << myInteger << std::endl;

    mongo::client::GlobalInstance instance;
    if (!instance.initialized()) {
        std::cout << "failed to initialize the client driver: " << instance.status() << std::endl;
        return EXIT_FAILURE;
    }
    try {


        std::string uri = QString(getenv("MONGO_URL")).toStdString();
        if(uri == "") {

            std::cerr << "MONGO_URL env not found" << std::endl;
            return 1;
        }

//                "mongodb://tim:ilovetim@ds023418.mlab.com:23418/goparse";
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

        dbName = QString::fromStdString(cs.getDatabase());

        std::cout << "Connected to database: "<< dbName.toStdString().c_str() << std::endl;


        std::cout << "connected ok to mongodb" << std::endl;
    } catch( const mongo::DBException &e ) {
        std::cout << "caught " << e.what() << std::endl;
    }


    std::cout << "Parsing Args..." << std::endl;
    cmdline::parser *parser = new cmdline::parser();

    parser->add<std::string>("dbname", 'c', "dbname",  false);
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

//    dbName = QString::fromStdString(parser->get<std::string>("dbname"));


    std::cout << "Args are: " << std::endl;
    std::cout << "render: " << render_it << "\n" ;
//    std::cout << "dbName: " << dbName.toStdString().c_str() << "\n" ;

    std::cout << "Finished Init..." << std::endl;

    return 0;
}

//This loop is called over and over again. We do 3 different things
// 1) First step: start the planner
// 2) Middle steps: step the planner
// 3) Last step, save the grasps
int GraspEvalPlugin::mainLoop()
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



// void GraspEvalPlugin::uploadResults()
// {

//     SearchEnergy *mEnergyCalculator = new SearchEnergy();
//     mEnergyCalculator->setType(ENERGY_CONTACT_QUALITY);
//     mEnergyCalculator->setContactType(CONTACT_PRESET);

//     int num_grasps = mPlanner->getListSize();
//     std::cout << "Found " << num_grasps << " Grasps. " << std::endl;
//     std::string mongoCollName = (dbName + QString(".grasps")).toStdString();
//     std::cout <<"Uploading to Mongo Coll: " << mongoCollName << std::endl;

//     for(int i=0; i < num_grasps; i++)
//     {
//         GraspPlanningState gps = mPlanner->getGrasp(i);
//         gps.execute(mHand);
//         mHand->autoGrasp(render_it, 1.0, false);
//         bool is_legal;
//         double new_planned_energy;

//         mEnergyCalculator->analyzeCurrentPosture(mHand,graspItGUI->getMainWorld()->getGB(0),is_legal,new_planned_energy,false );
//         gps.setEnergy(new_planned_energy);
//         gps.saveCurrentHandState();

//         graspItGUI->getIVmgr()->getViewer()->render();
//         //usleep(1000000);

//         BSONObj p = toMongoGrasp(&gps, QString("ENERGY_CONTACT_QUALITY"));

//         c->insert(mongoCollName, p);

//     }
//     // TODO: find a better way to die
//     assert(false);
// }

// mongo::BSONObj GraspEvalPlugin::toMongoGrasp(GraspPlanningState *gps, QString energyType)
// {
//     BSONObjBuilder grasp;
//     BSONObjBuilder pose;
//     BSONObjBuilder model;
//     BSONObjBuilder energy;
//     BSONArrayBuilder translation;
//     BSONArrayBuilder rotation;
//     BSONArrayBuilder dof;

//     Hand *hand = gps->getHand();
//     GraspableBody *body = gps->getObject();

//     double dofVals [hand->getNumDOF()];
//     hand->getDOFVals(dofVals);

//     transf hand_pose = mHand->getPalm()->getTran();

//     for(int dof_idx = 0; dof_idx < hand->getNumDOF(); dof_idx ++)
//     {
//         dof.append(dofVals[dof_idx]);
//     }

//     energy.append("type", "ENERGY_CONTACT_QUALITY");
//     energy.append("value", gps->getEnergy());

//     translation.append(hand_pose.translation().x()).append(hand_pose.translation().y()).append(hand_pose.translation().z());
//     rotation.append(hand_pose.rotation().w).append(hand_pose.rotation().x).append(hand_pose.rotation().y).append(hand_pose.rotation().z);
//     pose.append("translation", translation.arr());
//     pose.append("rotation", rotation.arr());

//     QString url = modelJson["url"].toString();
//     QString modelName = modelJson["name"].toString();
//     QString material = modelJson["material"].toString();
//     double dimension = modelJson["dimension"].toDouble();

//     model.append("name", modelName.toStdString());
//     model.append("url", url.toStdString());
//     model.append("material", material.toStdString());
//     model.append("dimension", dimension);

//     grasp.append("model", model.obj());
//     grasp.append("hand", hand->getDBName().toStdString());
//     grasp.append("energy", energy.obj());
//     grasp.appendArray("dof", dof.arr());
//     grasp.append("pose", pose.obj());

//     return grasp.obj();
// }

