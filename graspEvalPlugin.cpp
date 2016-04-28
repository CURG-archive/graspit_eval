#include "graspEvalPlugin.h"

#include "QJsonObject.h"
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>


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


GraspEvalPlugin::GraspEvalPlugin() {

}

GraspEvalPlugin::~GraspEvalPlugin() {
}        


int GraspEvalPlugin::init(int argc, char **argv) {

    std::cout << "Starting GraspEvalPlugin: " << std::endl ;
    std::cout << "Connecting to Mongo..." << std::endl ;


    mongo::client::GlobalInstance instance;
    if (!instance.initialized()) {
        std::cout << "failed to initialize the client driver: " << instance.status() << std::endl;
        return EXIT_FAILURE;
    }
    try {


        std::string uri = "mongodb://tim:ilovetim@ds023418.mlab.com:23418/goparse"; 
        //QString(getenv("MONGO_URL")).toStdString();
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


        BSONObjBuilder g;
        g.append("sample", "graspit");

        BSONObj p =g.obj();
        c->insert("goparse.helloworld", p);

    } catch( const mongo::DBException &e ) {
        std::cout << "caught " << e.what() << std::endl;
    }

    std::cout << "Finished Init..." << std::endl;

    return 0;
}

//This loop is called over and over again. We do 3 different things
// 1) First step: start the planner
// 2) Middle steps: step the planner
// 3) Last step, save the grasps
int GraspEvalPlugin::mainLoop()
{
   
    std::cout << "Inside the mainLoop()" << std::endl;
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

