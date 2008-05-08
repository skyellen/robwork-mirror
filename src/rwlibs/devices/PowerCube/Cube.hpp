#ifndef CUBE_HPP_
#define CUBE_HPP_

#include <rwlibs/io/canbus/CanPort.hpp>
#include <vector>

#include "PCubeProtocol.hpp"

namespace rwlibs { namespace devices {

    /** @addtogroup devices */
    /*@{*/

    /**
     * @brief Implementation of the Cube protocol. It is basicly a interface to communicate 
     * with Cube devices, f.ex. the shunck parallel gripper PA70.
     */
    class Cube {

    public:
        //! Cube baud rates
        typedef enum { CUBE_250K_BAUD = 1, CUBE_500K_BAUD, CUBE_1000K_BAUD } CubeBaudrate;

    private:
        /**
         * @brief default constructor
         */
        Cube( int moduleId, rwlibs::io::CanPort *port );

    public:
        /**
         * @brief default destructor
         */
        ~Cube();

    public:

        // global function calls
        /**
         * @brief tests wether a cube on port p is connected to module with id moduleNr
         */
        static bool isCubeConnected( int moduleNr, rwlibs::io::CanPort* p );

        /**
         * @brief returns a cube object handle with module id mId on port p if this is
         * connected.
         */
        static Cube* getCubeAt( int mId, rwlibs::io::CanPort* p);

        /**
         * @brief gets all handles of cubes in the range [from,to] if the cube is
         * detected/connected 
         */
        static std::vector<Cube*> getCubes(size_t from, size_t to, rwlibs::io::CanPort* p);
        
        /**
         * @brief emits a reset all command. All cubes connected to port p is reset.
         */
        static void resetAllCmd( rwlibs::io::CanPort* p );

        /**
         * @brief emits a home all command. All cubes connected to port p is homed.
         */
        static void homeAllCmd( rwlibs::io::CanPort* p );

        /**
         * @brief emits a halt all command. All cubes connected to port p is halted.
         */
        static void haltAllCmd( rwlibs::io::CanPort* p );

        /**
         * @brief emits a watchdog refreash all command. All cubes connected to port p
         * will have their watchdog refreashed.
         */
        static void watchdogAllCmd( rwlibs::io::CanPort* p );

        /**
         * @brief Sets the baudrate for all cubes connected to port p.
         */
        static void setBaudrateAll( CubeBaudrate rate, rwlibs::io::CanPort* p );

        static void savePosAllCmd( rwlibs::io::CanPort* p );
        static void syncMotionAllCmd( rwlibs::io::CanPort* p );

        // generel non communicating methods
        /**
         * @brief gets the module id of the physical cube associated to 
         * this handle
         */
        int getCubeID(){ return _moduleId; }
        
        // Reset, home and halt commands.
        void resetCmd();
        void homeCmd();
        void haltCmd();

        // *********** motion commands ************

        void moveRampCmd(float val); // Ramp to the position val.
        void moveStepCmd(float pos, double time); // The time is in seconds.
        void moveVelCmd(float val); // Velocity in m/s.
        void moveCurCmd(float val); // Current in Ampere.
        void moveRampTicksCmd(int val);
        void moveStepTicksCmd(int pos, double time); // Encoder ticks per second.
        void moveVelTicksCmd(int val);
        void moveCurTicksCmd(int val); // Current in ??

        // *********** motion commands with extended acknoledge ********

        float moveRampExtCmd(float val); // Ramp to the position val.
        float moveStepExtCmd(float pos, double time); // The time is in seconds.
        float moveVelExtCmd(float val); // Velocity in m/s.
        float moveCurExtCmd(float val); // Current in Ampere.
        int moveRampTicksExtCmd(int val);
        int moveStepTicksExtCmd(int pos, double time); // Encoder ticks per second.
        int moveVelTicksExtCmd(int val);
        int moveCurTicksExtCmd(int val); // Current in ??

        // *********** parameter commands *******************
        float getDefHomeOffset(){ return getFloatParam( PCUBE_DefHomeOffset ); };
        float getDefGearRatio(){ return getFloatParam( PCUBE_DefGearRatio ); };
        float getDefLinRatio(){ return getFloatParam( PCUBE_DefLinRatio); };
        float getDefMinPos(){ return getFloatParam( PCUBE_DefMinPos); };
        float getDefMaxPos(){ return getFloatParam( PCUBE_DefMaxPos); };
        float getDefMaxDeltaPos(){ return getFloatParam( PCUBE_DefMaxDeltaPos); };
        float getDefMaxDeltaVel(){ return getFloatParam( PCUBE_DefMaxDeltaVel); };
        float getDefTorqueRatio(){ return getFloatParam( PCUBE_DefTorqueRatio); };
        float getDefCurRatio(){ return getFloatParam( PCUBE_DefCurRatio); };
        float getDefMinVel(){ return getFloatParam( PCUBE_DefMinVel); };
        float getDefMaxVel(){ return getFloatParam( PCUBE_DefMaxVel); };
        float getDefMinAcc(){ return getFloatParam( PCUBE_DefMinAcc); };
        float getDefMaxAcc(){ return getFloatParam( PCUBE_DefMaxAcc); };
        float getDefMinCur(){ return getFloatParam( PCUBE_DefMinCur); };
        float getDefMaxCur(){ return getFloatParam( PCUBE_DefMaxCur); };
        float getDefHomeVel(){ return getFloatParam( PCUBE_DefHomeVel); };
        float getDefHomeAcc(){ return getFloatParam( PCUBE_DefHomeAcc); };

        unsigned int getDefCubeSerial(){ return getInt32Param( PCUBE_DefHomeOffset); };
        unsigned int getDefConfig(){ return getInt32Param( PCUBE_DefHomeOffset); };
        unsigned int getDefPulsesPerTurn(){ return getInt32Param( PCUBE_DefHomeOffset); };

        unsigned int getDefCubeVersion(){ return getInt16Param( PCUBE_DefHomeOffset); };
        unsigned int getDefServiceInterval(){ return getInt16Param( PCUBE_DefHomeOffset); };
        unsigned int getDefBrakeTimeOut(){ return getInt16Param( PCUBE_DefHomeOffset); };

        unsigned char getDefAddress(){ return getInt8Param( PCUBE_DefAddress); };
        unsigned char getDefPrimBaud(){ return getInt8Param( PCUBE_DefPrimBaud); };
        unsigned char getDefScndBaud(){ return getInt8Param( PCUBE_DefScndBaud); };

        int getPosCount(){ return getInt32Param( PCUBE_DefHomeOffset); };
        int getRefPosCount(){ return getInt32Param( PCUBE_DefHomeOffset); };

        unsigned int getDioSetup(){ return getInt32Param( PCUBE_DefHomeOffset); };
        void setDioSetup( unsigned int setup);

        unsigned int getCubeState(){ return getInt32Param( PCUBE_DefHomeOffset); };

        unsigned int getTargetPosInc(){ return getInt32Param( PCUBE_DefHomeOffset); };
        void setTargetPosInc(unsigned int val );
        unsigned int getTargetVelInc(){ return getInt32Param( PCUBE_DefHomeOffset); };
        void setTargetVelInc(unsigned int val );
        unsigned int getTargetAccInc(){ return getInt32Param( PCUBE_DefHomeOffset); };
        void setTargetAccInc(unsigned int val );

        unsigned int getStepInc(){ return getInt32Param( PCUBE_DefHomeOffset); };

        int getHomeOffsetInc(){ return getInt32Param( PCUBE_HomeOffsetInc); };

        int getRawCur(){ return getInt32Param( PCUBE_RawCur); };
        void setRawCur(int);

        int getHomeToZeroInc(){ return getInt32Param( PCUBE_HomeToZeroInc); };

        unsigned int getConfig(){ return getInt32Param( PCUBE_Config); };
        void setConfig(unsigned int val );

        unsigned char getMoveMode(){ return getInt8Param( PCUBE_MoveMode); };

        float getIncRatio();

        float getActPos(){ return getFloatParam( PCUBE_ActPos ); };
        float getActPosPrev(){ return getFloatParam( PCUBE_ActPosPrev ); };
        float getIPolPos(){ return getFloatParam( PCUBE_IPolPos ); };
        float getDeltaPos(){ return getFloatParam( PCUBE_DeltaPos ); };
        float getMaxDeltaPos(){ return getFloatParam( PCUBE_MaxDeltaPos ); };
        void setMaxDeltaPos( float val ){ setFloatParam( PCUBE_MaxDeltaPos, val ); };
        float getActVel(){ return getFloatParam( PCUBE_ActVel ); };
        float getIPolVel();

        float getMinPos(){ return getFloatParam(PCUBE_MinPos); };
        void setMinPos( float val ){ setFloatParam(PCUBE_MinPos, val); };
        float getMaxPos(){ return getFloatParam(PCUBE_MaxPos); };
        void setMaxPos( float val ){ setFloatParam(PCUBE_MaxPos, val); };
        float getMaxVel(){ return getFloatParam(PCUBE_MaxVel); };
        void setMaxVel( float val ){ setFloatParam(PCUBE_MaxVel, val); };
        float getMaxAcc(){ return getFloatParam(PCUBE_MaxAcc); };
        void setMaxAcc( float val ){ setFloatParam(PCUBE_MaxAcc, val); };
        float getMaxCur(){ return getFloatParam(PCUBE_MaxCur); };
        void setMaxCur( float val ){ setFloatParam(PCUBE_MaxCur, val); };

        
        float getCur(){ return getFloatParam(PCUBE_Cur); };
        void setCur(float val ){ setFloatParam(PCUBE_Cur, val); };

        void setTargetPos( float val ){ setFloatParam(PCUBE_TargetPos, val); };
        void setTargetVel( float val ){ setFloatParam(PCUBE_TargetVel, val); };
        void setTargetAcc( float val ){ setFloatParam(PCUBE_TargetAcc, val); };

        /* Some missing here */

        unsigned int getSetup();
        float getHomeOffset();
        void setHomeOffset( float val );

    private:
        bool ack(const Cmd& cmd);
        bool ackext(const Cmd& cmd, rwlibs::io::CanPort::CanMessage& msg);
        void emitCmd( const Cmd& org_cmd );

        void setFloatParam( unsigned char paramid, float paramval );
        void setInt32Param( unsigned char paramid, int val);
        
        float getFloatParam( unsigned char paramid );
        int getInt32Param( unsigned char paramid );
        int getInt16Param( unsigned char paramid );
        char getInt8Param( unsigned char paramid );

        int cmdIdAck() {
            return PCUBE_CANID_CMDACK + _moduleId;
        }

        int cmdIdGet() {
            return PCUBE_CANID_CMDGET + _moduleId;
        }

        int cmdIdPut() {
            return PCUBE_CANID_CMDPUT + _moduleId;
        }

        const int _moduleId;
        rwlibs::io::CanPort* _port;
        rwlibs::io::CanPort& getPort() const { return *_port; }

    private:
        Cube(const Cube&);
        Cube& operator=(const Cube&);
    };

    /*@}*/
    
} // namespace devices
} // namespace rwlibs

#endif /*CUBE_HPP_*/
