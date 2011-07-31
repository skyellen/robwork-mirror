#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/Motion.hpp>
#include <rwlibs/task/Action.hpp>
#include <rwlibs/task/Target.hpp>
#include <rwlibs/task/loader/XMLTaskLoader.hpp>
#include <rwlibs/task/loader/XMLTaskSaver.hpp>
#include <iostream>

using namespace rw::math;
using namespace rw::common;
using namespace rwlibs::task;

void printMotion(QMotionPtr motion) {
    switch (motion->motionType()) {
    case MotionType::P2P: {
        QP2PMotionPtr p2p = motion.cast<QP2PMotion>();
        std::cout<<"Got P2P Motion from "<<p2p->start()<<" to "<<p2p->end()<<std::endl;
        break; }
    case MotionType::Linear: {
        QLinearMotionPtr lin = motion.cast<QLinearMotion>();
        std::cout<<"Got Linear Motion from "<<lin->start()<<" to "<<lin->end()<<std::endl;
        break; }
    }
}

void printAction(ActionPtr action) {
    std::cout<<"Got Action of type = "<<action->getId()<<std::endl;
}

void printTask(QTaskPtr task) {
    std::vector<EntityPtr> entities = task->getEntities();
    for (std::vector<EntityPtr>::iterator it = entities.begin(); it != entities.end(); ++it) {
        EntityPtr entity = *it;
        switch (entity->entityType()) {
        case EntityType::Motion: {
            QMotionPtr motion = entity.cast<QMotion>();
            printMotion(motion);
            break; }
        case EntityType::Action: {
            ActionPtr action = entity.cast<Action>();
            printAction(action);
            break; }
        case EntityType::Task: {
            QTaskPtr task = entity.cast<QTask>();
            printTask(task);
            break; }
        }
    }
}

int main(int argc, char* argv[]) {
    //Construct a Task
    QTaskPtr task = ownedPtr(new QTask());
    rw::math::Q q1(1); q1(0) = 1;
    rw::math::Q q2(1); q2(0) = 2;
    rw::math::Q q3(1); q3(0) = 3;
    task->addTargetByValue(q1);
    task->addTargetByValue(q2);
    task->addTargetByValue(q3);

    std::vector<QTargetPtr>& targets = task->getTargets();
    task->addMotion(ownedPtr(new QP2PMotion(targets[0], targets[1])));
    task->addMotion(ownedPtr(new QLinearMotion(targets[1], targets[2])));
    task->addAction(ownedPtr(new Action(ActionType::On)));

    printTask(task);

    XMLTaskSaver::save(task, "MyTask.xml");

    XMLTaskLoader loader;
    loader.load("MyTask.xml");
    QTaskPtr task2 = loader.getQTask();

    printTask(task2);

    return 0;
}
