#include "graspEvalPlugin.h"

#include "QJsonObject.h"
#include <cmath>
#include <fstream>

#include <include/world.h>
#include <include/body.h>
#include <include/robot.h>
#include <include/graspitGUI.h>
#include <include/ivmgr.h>
#include <ui/mainWindow.h>


#include <include/grasp.h>
#include <include/triangle.h>
#include <include/debug.h>

#include <cmdline/cmdline.h>
#include "include/dbModelLoader.h"

#include <cstdlib>
#include <iostream>

#include "mongo/client/dbclient.h" // for the driver

#include <include/EGPlanner/searchState.h>

using mongo::BSONArray;
using mongo::BSONArrayBuilder;
using mongo::BSONObj;
using mongo::BSONObjBuilder;
using mongo::BSONElement;

using namespace mongo;


GraspEvalPlugin::GraspEvalPlugin():
    hasAutoGrasped(false),
    hasApproachedTilContact(false),
    hasLifted(false),
    step_count(0),
    mHand(NULL)
{

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


        std::string uri = "mongodb://localhost:27017/test";
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

        getGrasps(c);

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
    step_count++;

    if(mHand == NULL)
    {
        mHand = graspItGUI->getMainWorld()->getCurrentHand();
    }

    if(!graspItGUI->getMainWorld()->dynamicsAreOn())
    {
         graspItGUI->getMainWorld()->turnOnDynamics();
    }

//    if(!hasApproachedTilContact)
//    {
//        graspItGUI->getMainWorld()->getCurrentHand()->approachToContact(100, false);
//        hasApproachedTilContact = true;
//        std::cout << " hasApproachedTilContact step_count: " << step_count << std::endl;
//    }

//    if(hasApproachedTilContact && !hasAutoGrasped && step_count > 100)
//    {
//        graspItGUI->getMainWorld()->getCurrentHand()->autoGrasp(true);
//        graspItGUI->getMainWorld()->updateGrasps();
//        hasAutoGrasped=true;
//        std::cout << " hasAutoGrasped step_count: " << step_count << std::endl;
//    }

//    if(hasAutoGrasped && !hasLifted&& step_count > 500)
//    {
        std::cout << "trying to lift" << std::endl;
        liftHand(1000,false);
        //hasLifted=true;
         std::cout << "hasLifted step_count: " << step_count << std::endl;
//    }

    return 0;
}

bool GraspEvalPlugin::liftHand(double moveDist, bool oneStep)
{
//    double newvelocity[6];
//    const double* velocity = mHand->getPalm()->getVelocity();
//    transf newTran =  translate_transf(vec3(0,0,step_count*.1) * mHand->getTran());
//    mHand->setTran(newTran);
//    std::vector<Body*> bodies;
//    mHand->getBodyList(&bodies);
//    for(int i = 0; i < bodies.size(); i++)
//    {
//        double newvelocity[6];
//        Body *b = bodies.at(i);
//        DynamicBody *db = dynamic_cast<DynamicBody*> (b);
////        const double* velocity = db->getVelocity();

////            newvelocity[0] = velocity[0];
////            newvelocity[1] = velocity[1];
////            newvelocity[2] = velocity[2];
////            newvelocity[3] = velocity[3];
////            newvelocity[4] = velocity[4];
////            newvelocity[5] = velocity[5];
////            newvelocity[2] += step_count;
////            db->setVelocity(newvelocity);
//            std::cout << "Here I am" << db->isDynamic() <<  std::endl;
//            db->addForce(vec3(0,0,1000));
//    }
    Link *p = mHand->getPalm();
     DynamicBody *db = dynamic_cast<DynamicBody*> (p);
     db->addForceAtPos(vec3(0,0,1000000000), position(0,0,0));
     db->setUseDynamics(true);
    //graspItGUI->getMainWorld()->moveDynamicBodies(1/60.0);
    graspItGUI->getIVmgr()->getViewer()->render();
//    newvelocity[0] = velocity[0];
//    newvelocity[1] = velocity[1];
//    newvelocity[2] = velocity[2];
//    newvelocity[3] = velocity[3];
//    newvelocity[4] = velocity[4];
//    newvelocity[5] = velocity[5];
//    newvelocity[0] += 100;
//    mHand->getPalm()->setVelocity(newvelocity);
//    transf newTran = translate_transf(vec3(0,moveDist,0) * mHand->getApproachTran()) * translate_transf(vec3(0,moveDist,0) * mHand->getTran());
//    bool result;
//    std::cout << "about to lift hand" << std::endl;
//    if (oneStep) {
//        result = mHand->moveToIgnoreCollision(newTran, WorldElement::ONE_STEP, WorldElement::ONE_STEP);
//    } else {
//        result = mHand->moveToIgnoreCollision(newTran, .1*Contact::THRESHOLD, M_PI/36.0);
//    }
//    if (result) {
//        DBGP("Approach no contact");
//        return false;
//    } else {
//        DBGP("Approach results in contact");
//        return true;
//    }
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


void GraspEvalPlugin::getGrasps(mongo::DBClientBase *c) {


    std::string collName = "grasps";

    std::string mongoCollName = (dbName + "." + QString::fromStdString(collName)).toStdString();


    std::auto_ptr<DBClientCursor> cursor = c->query(mongoCollName, BSONObj());
    BSONObj graspBsonObj;
    while (cursor->more()) {
        graspBsonObj = cursor->next();

        QString modelUrl = QString::fromStdString(graspBsonObj.getField("model").Obj().getField("url").valueStringData().toString());

        QString modelName = QString::fromStdString(graspBsonObj.getField("model").Obj().getField("name").valueStringData().toString());

        QString handName = QString::fromStdString(graspBsonObj.getField("hand").valueStringData().toString());

        std::cout<< "Hand: " << handName.toStdString().c_str() << std::endl;

        std::cout<< "modelName: " << modelName.toStdString().c_str() << std::endl;

        std::cout<< "modelUrl: " << modelUrl.toStdString().c_str() << std::endl;

        double dof = graspBsonObj.getField("dof").Array().front().Double();
        std::cout<< "dof: " <<  dof << std::endl;

        //get this from mongo, need to find-replace all mongo robot names to pr2_gripper_2010
//        QString robotname = QString("pr2_gripper_2010");
        QString robotPath= QString(getenv("GRASPIT")) + QString("/models/robots/") + handName + QString("/") +  handName + ".xml";


//         std::cout<< "robot path: " << robotFilepath.toStdString().c_str() << std::endl;
//        mHand = static_cast<Hand* >(graspItGUI->getMainWorld()->importRobot(robotFilepath));

        std::cout<< "robot path: " << robotPath.toStdString().c_str() << std::endl;

        DbModelLoader loader;
        graspItGUI->getMainWorld()->importRobot(robotPath.toStdString().c_str());

        loader.loadModelFromUrl(modelUrl, modelName, QString("rubber"), 150.0);
        break;

//        mHand = graspItGUI->getMainWorld()->getCurrentHand();


        // TODO; use the load from url option in dbModelLoader
        //get this from mongo,
//        QString bodyfilename = QString("mug.off");
        // Use dbModelLoader to load the body from off
//        Body* b = graspItGUI->getMainWorld()->importBody("GraspableBody", bodyfilename);

//        GraspPlanningState *gps = new GraspPlanningState(mHand);
//        gps->setObject(b);

//       std::cout << graspBsonObj.toString() << std::endl;
    }

}
