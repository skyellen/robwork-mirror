#ifndef TASK2_HPP
#define TASK2_HPP


#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>
#include <rtt/Command.hpp>
#include <rtt/Method.hpp>


namespace Template
{
    
  class Task2 : public RTT::TaskContext
  {
  public:
      /// constructor
      Task2(std::string name);

      /// destructor
      virtual ~Task2();
    
      /// startup function, called once on start
      virtual bool startup();

      /// update function, called every time the activity triggers the task
      virtual void update();

      /// shutdown function, called once on shutdown
      virtual void shutdown();

  private:

      // methods of this task
      RTT::Method<int(void)> _getCounter;

    
      RTT::ReadDataPort<int> _counterPort;

      // method
      int getCounter();
      

   }; // class
} // namespace

#endif
