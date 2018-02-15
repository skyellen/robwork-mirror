#include <rwlibs/task/Task.hpp>
#include <rwlibs/task/Motion.hpp>
#include <rwlibs/task/Action.hpp>
#include <rwlibs/task/Target.hpp>
#include <rwlibs/task/loader/TaskLoader.hpp>
#include <rwlibs/task/loader/TaskSaver.hpp>
#include <iostream>

using namespace rw::math;
using namespace rw::common;
using namespace rwlibs::task;

void printMotion(QMotion::Ptr motion) {
    switch (motion->motionType()) {
    case MotionType::P2P: {
        QP2PMotion::Ptr p2p = motion.cast<QP2PMotion>();
        std::cout<<"Got P2P Motion from "<<p2p->start()<<" to "<<p2p->end()<<std::endl;
        break; }
    case MotionType::Linear: {
        QLinearMotion::Ptr lin = motion.cast<QLinearMotion>();
        std::cout<<"Got Linear Motion from "<<lin->start()<<" to "<<lin->end()<<std::endl;
        break; }
    }
}

void printAction(Action::Ptr action) {
    std::cout<<"Got Action of type = "<<action->getId()<<std::endl;
}

void printTask(QTask::Ptr task) {
    std::vector<Entity::Ptr> entities = task->getEntities();
    for (std::vector<Entity::Ptr>::iterator it = entities.begin(); it != entities.end(); ++it) {
        Entity::Ptr entity = *it;
        switch (entity->entityType()) {
        case EntityType::Motion: {
            QMotion::Ptr motion = entity.cast<QMotion>();
            printMotion(motion);
            break; }
        case EntityType::Action: {
            Action::Ptr action = entity.cast<Action>();
            printAction(action);
            break; }
        case EntityType::Task: {
            QTask::Ptr task = entity.cast<QTask>();
            printTask(task);
            break; }
        }
    }
}

int main() {
    //Construct a Task
    QTask::Ptr task = ownedPtr(new QTask());
    rw::math::Q q1(1); q1(0) = 1;
    rw::math::Q q2(1); q2(0) = 2;
    rw::math::Q q3(1); q3(0) = 3;
    task->addTargetByValue(q1);
    task->addTargetByValue(q2);
    task->addTargetByValue(q3);

    std::vector<QTarget::Ptr>& targets = task->getTargets();
    task->addMotion(ownedPtr(new QP2PMotion(targets[0], targets[1])));
    task->addMotion(ownedPtr(new QLinearMotion(targets[1], targets[2])));
    task->addAction(ownedPtr(new Action(ActionType::On)));

    printTask(task);

    TaskSaver::Ptr saver = TaskSaver::Factory::getTaskSaver("xml");
    saver->save(task, "MyTask.xml");

    TaskLoader::Ptr loader = TaskLoader::Factory::getTaskLoader("xml");
    loader->load("MyTask.xml");
    QTask::Ptr task2 = loader->getQTask();

    printTask(task2);

    return 0;
}
