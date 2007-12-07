#ifndef TASK1_HPP
#define TASK1_HPP

#include <rtt/RTT.hpp>
#include <rtt/TaskContext.hpp>
#include <rtt/Ports.hpp>
#include <rtt/Properties.hpp>
#include <rtt/Command.hpp>
#include <rtt/Method.hpp>


namespace Template
{
    
  class Task1 : public RTT::TaskContext
  {
  public:
      /// constructor
      Task1(std::string name);

      /// destructor
      virtual~Task1();
    
      /// startup function, called once on start 
      virtual bool startup();

      /// update function, called every time the activity triggers the task
      virtual void update();

      /// shutdown function, called once on shutdown
      virtual void shutdown();

  private:

      // method
      RTT::Method<void(int)> _setCounter;

      // command
      RTT::Command<bool(int)> _command1;

      // port
      RTT::WriteDataPort<int> _counterPort;

    
      // method functions
      void setCounter(int counter);
      
      // command functions
      bool command1(int b);
      bool command_completion() const;


   }; // class
} // namespace

#endif
