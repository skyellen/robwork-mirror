#ifndef PCUBEPROTOCOL_HPP_
#define PCUBEPROTOCOL_HPP_

#include <vector>


// ******************** CAN ID for commands

#define PCUBE_CANID_CMDACK 0x0a0
#define PCUBE_CANID_CMDGET 0x0c0
#define PCUBE_CANID_CMDPUT 0x0e0
#define PCUBE_CANID_CMDALL 0x100


// ******************** Command ALL id's for the powercube

#define PCUBE_ResetAll 0x00 // 1 Byte, Clear error state
#define PCUBE_HomeAll 0x01 // 1 Byte, Start Homing procedure
#define PCUBE_HaltAll 0x02 // 1 Byte, Stop immediately
#define PCUBE_WatchdogRefreshAll 0x07
#define PCUBE_SetBaudAll 0x09
#define PCUBE_SavePosAll 0x0e
#define PCUBE_SyncMotionAll 0x0f



// ******************** Command id's for the powercube

#define PCUBE_Reset 0x00 // 1 Byte, Clear error state
#define PCUBE_Home 0x01 // 1 Byte, Start Homing procedure
#define PCUBE_Halt 0x02 // 1 Byte, Stop immediately
#define PCUBE_RecalcPIDParam 0x09 // 1 Byte, Recalculate the PID loop parameters
#define PCUBE_SetExtended 0x08 // 3-6 Byte, Set parameter
#define PCUBE_GetExtended 0x0a // 2 Byte, Fetch parameter
#define PCUBE_SetMotion 0x0b // 6-8 Byte, Set Motion command
#define PCUBE_SetIStep 0x0d // 7 Byte, Motion command in Step mode

/* Motion id's for the powercobe. To be used in conjunction
 * with the motion command SetMotion: [0x0b][motionId]...
 */

// motion commands with simple ack, [0x0b][motionId][0x64]

#define PCUBE_FRAMP_MODE 0x04  // Target position float rad resp. m
#define PCUBE_FSTEP_MODE 0x06  // Target position Time, float UInt16, rad resp. m ms
#define PCUBE_FVEL_MODE 0x07 // Velocity float rad/s resp. m/s.
#define PCUBE_FCUR_MODE 0x08 // Sollstrom float A.
#define PCUBE_IRAMP_MODE 0x09 // Target position Int32 Encoder ticks.
#define PCUBE_ISTEP_MODE 0x0b // Target position Time, Int32 UInt16, Encoder ticks. ms.
#define PCUBE_IVEL_MODE 0x0c // Velocity Int32 Encoder ticks/s.
#define PCUBE_ICUR_MODE 0x0d // Current Int16 Digits.

// motion commands with extended ack, [0x0b][motionId][position][state][dio]

#define PCUBE_FRAMP_ACK 0x0e //Target position float rad resp. m.
#define PCUBE_FSTEP_ACK 0x10 //Zielposition Zeitvorgabe, float UInt16, rad resp. m. ms.
#define PCUBE_FVEL_ACK 0x11 //Velocity float rad/s resp. m/s.
#define PCUBE_FCUR_ACK 0x12 //Current float A.
#define PCUBE_IRAMP_ACK 0x13 // Target position Int32 Encoder ticks.
#define PCUBE_ISTEP_ACK 0x15 // Target position Time, Int32 UInt16, Encoder ticks. ms.
#define PCUBE_IVEL_ACK 0x16 // Velocity Int32 Encoder ticks/s.
#define PCUBE_ICUR_ACK 0x17 // Current Int16 Digits.

/* Short state in acknoledge
 * The short module state transfered in the acknowledge of a motion command is based on the
 * full module state word.
 */

#define PCUBE_SHORT_NOT_OK 0x01 // (STATE_ERROR) OR (NOT STATE_HOME_OK) OR (STATE_HALTED)
#define PCUBE_SHORT_SWR 0x02 // STATE_SWR
#define PCUBE_SHORT_SW1 0x04 // STATE_SW1
#define PCUBE_SHORT_SW2 0x08 // STATE_SW2
#define PCUBE_SHORT_MOTION 0x10 // STATE_MOTION
#define PCUBE_SHORT_RAMP_END 0x20  // STATE_RAMP_END
#define PCUBE_SHORT_INPROGRESS 0x40  // STATE_INPROGRESS
#define PCUBE_SHORT_FULLBUFFER 0x80 // STATE_FULLBUFFER

/*
 * Digital IO state in Acknowledge
 * The last data byte of the acknowledge to a motion command contains the digital IO state. The
 * byte is organised like this:
 */
#define PCUBE_INBIT0 0x01 // State of input 0
#define PCUBE_INBIT1 0x02 // State of input 1
#define PCUBE_INBIT2 0x04 // State of input 2
#define PCUBE_INBIT3 0x08  // State of input 3
#define PCUBE_OUTBIT0 0x10 // State of output 0
#define PCUBE_OUTBIT1 0x20 // State of output 1
#define PCUBE_OUTBIT2 0x40 // State of output 2
#define PCUBE_OUTBIT3 0x80 // State of output 3

/*
 *  Parameter ID's
 * This table contains all module parameters available to the user:
 */

#define PCUBE_DefHomeOffset 0 // 0x00  Offset to the Home position (Default value). By means of
                             // this parameter teh user can adjust an offset to the
                             // physical zero position (home) of the drive.
                             // float (read)

#define PCUBE_DefGearRatio 1 // 0x01 // Gear ratio (Default value) float x
#define PCUBE_DefLinRatio 2 // 0x02 // Transmission factor for transforming rotary in linear motion (Default value)
                           // float (read)
#define PCUBE_DefMinPos 3 // 0x03 // Minimum drive position (Default value) float x
#define PCUBE_DefMaxPos 4 // 0x04 // Maximum drive position (Default value) float x
#define PCUBE_DefMaxDeltaPos 5 // 0x05  Maximum following error (tow) for the digital servo filter (Default value)
                              // float x
#define PCUBE_DefMaxDeltaVel 6 // 0x06  Maximum following error for velocity control (Default value)
                              // float (read)
#define PCUBE_DefTorqueRatio 7 // 0x07  Transmission factor for transforming current to torque
                              // (Default value) float (read)
#define PCUBE_DefCurRatio 8 // 0x08 Transmission factor for current measurement (Default value) float x
#define PCUBE_DefMinVel 9 // 0x09 Minimum velocity (Default value) float x
#define PCUBE_DefMaxVel 10 // 0x0a Maximum velocity (Default value) float x
#define PCUBE_DefMinAcc 11 // 0x0b Minimum acceleration (Default value) float x
#define PCUBE_DefMaxAcc 12 // 0x0c Maximum acceleration (Default value) float x
#define PCUBE_DefMinCur 13 // 0x0d Minimum current (Default value) float x
#define PCUBE_DefMaxCur 14 // 0x0e Maximum current (Default value) float x
#define PCUBE_DefHomeVel 15 // 0x0f Homing velocity. This value is signed and thereby
                      // specifies the homing direction. (Default value) float x
#define PCUBE_DefHomeAcc 16 // 0x10 Homing acceleration (Default value) float x
#define PCUBE_DefCubeSerial 26 // 0x1a Serial number of the Plustronik (Default value) UInt32 x
#define PCUBE_DefConfig 27 // 0x1b Config word (Default value) UInt32 x
#define PCUBE_DefPulsesPerTurn 28 // 0x1c Number of Encoder ticks per revolution (Default value) UInt32 x
#define PCUBE_DefCubeVersion 29 // 0x1d Version information (Default value) UInt16 x
#define PCUBE_DefServiceInterval 30 // 0x1e Service interval (Default value) UInt16 x
#define PCUBE_DefBrakeTimeOut 31 // 0x1f Delay for releasing the brake in ms (Default value) UInt16 x
#define PCUBE_DefAddress 32 // 0x20 Module bus address [1...31] (Default value) UInt8 x
#define PCUBE_DefPrimBaud 34 // 0x22 Primary Baud rate setting (Default value) UInt8 x
#define PCUBE_DefScndBaud 35 // 0x23 Secondary Baud rate setting (Default value) UInt8 x
#define PCUBE_PosCount 36 // 0x24 Absolute Counter value (Actual value) Int32 x
#define PCUBE_RefPosCount 37 // 0x25 Absolute Counter value at Homing position (Actual value) Int32 x
#define PCUBE_DioSetup 38 // 0x26 Digital IO word UInt 32 x x
#define PCUBE_CubeState 39 // 0x27 Module State word (Actual value) UInt32 x
#define PCUBE_TargetPosInc 40 // 0x28 Target position in Encoder ticks (Target value) UInt32 x x
#define PCUBE_TargetVelInc 41 // 0x29 Target velocity in Encoder ticks/s (Target value) UInt32 x x
#define PCUBE_TargetAccInc 42 // 0x2a Target accedleration in Encoder ticks/s² (Target value) UInt32 x x
#define PCUBE_StepInc 43 // 0x2b Step mode target position in Encoder ticks (Actual value) UInt32 x
#define PCUBE_HomeOffsetInc 44 // 0x2c Home offset in Encoder ticks (Actual value) Int32 x
#define PCUBE_RawCur 53 // 0x35 Commanded Current in Digits [-500...+500] (Actual value) Int16 x x
#define PCUBE_HomeToZeroInc 54 // 0x36 Number of Encoder ticks between home switch and
                        // Encoder index (Actual value) Int32 x
#define PCUBE_Config 57 // 0x39 Config word (Vorgabe) UInt32 x x
#define PCUBE_MoveMode 58 // 0x3a Motion mode (Actual value) UInt8 x
#define PCUBE_IncRatio 59 // 0x3b Ration of Encoder ticks and unit, rad resp. m (Actual float x

#define PCUBE_ActPos 60 // 0x3c Actual position in rad resp. m (Actual value) float x
#define PCUBE_ActPosPrev 61 // 0x3d Previous position in rad resp. m (Actual value) float x
#define PCUBE_IPolPos 62 // 0x3e Actual interpolated position (Actual value) float x
#define PCUBE_DeltaPos 63 // 0x3f Actual following error (Actual value) float x
#define PCUBE_MaxDeltaPos 64 // 0x40 Maximum following error (Limit) float x x
#define PCUBE_ActVel 65 // 0x41 Actual velocity in units/s (Actual value) float x
#define PCUBE_IPolVel 66 // 0x42 Actual interpolated velocity in units/s (Actual value) float x
#define PCUBE_MinPos 69 // 0x45 Minimum position (Limit) float x x
#define PCUBE_MaxPos 70 // 0x46 Maximum position (Limit) float x x
#define PCUBE_MaxVel 72 // 0x48 Maximum velocity in units/s (Limit) float x x
#define PCUBE_MaxAcc 74 // 0x4a Maximum acceleration in units/s² (Limit) float x x
#define PCUBE_MaxCur 76 // 0x4c Maximum Current (Limit) float x x
#define PCUBE_Cur 77 // 0x4d Actual current (Actual value) float x x
#define PCUBE_TargetPos 78 // 0x4e Target position in units/s (Target value) float x
#define PCUBE_TargetVel 79 // 0x4f Target velocity in units/s (Target value) float x
#define PCUBE_TargetAcc 80 // 0x50 Target acceleration in units/s² (Target value) float x
#define PCUBE_DefC0 81 // 0x51 Servo loop gain C0 (Default value) Int16 x
#define PCUBE_DefDamp 82 // 0x52 Servo loop damping (Default value) Int16 x
#define PCUBE_DefA0 83 // 0x53 Servo loop parameter A0 (Default value) Int16
#define PCUBE_ActC0 84 // 0x54 Servo loop gain C0 (Actual value) Int16 x x
#define PCUBE_ActDamp 85 // 0x85 Servo loop damping (Actual value) Int16 x x
#define PCUBE_ActA0 86 // 0x86 Servo loop parameter A0 (Actual value) Int16 x x
#define PCUBE_DefBurnCount 87 // 0x87 Number of flash downloads (Default value) UInt8 x
#define PCUBE_Setup 88 // 0x88 Setup word (Default value) UInt32 x
#define PCUBE_HomeOffset 89 // 0x89 Home offset (Actual value) float x x

/**
 * Module state defines:
 * The module state can be permanently observed using the "CubeState" word. By means of the
 * state word the user receives error messages as well as information on the execution state of
 * motion commands. It is recommended to poll the state word on a regular basis. The
 * application controls the frequency for updating the data. The state word is an unsigned integer
 * val ue where each bit is treated as a flag. More than one flag can be set at a time:
 */


#define STATE_ERROR 0x00000001 // An error occured. The module stop immediately and does not accept motion
                               // commands anymore. The reason for the error state can be found reading the error
                               // flags. In many cases the error state can be reset by the user sending a Reset command.
                               // After a successful Reset the module is ready again to accept motion commands.
#define STATE_HOME_OK 0x00000002 // This flag is set after a successful homing procedure. It means that the drive has
                                 // successfully found its zero position. All limitations for the operation range are valid
                                 // now. If the user sends another Home-Command the flag will be reset until the
                                 // homing procedure has been finished successfully.
#define STATE_HALTED 0x00000004 // This flag is set in conjunction with an emergency stop. It means that the cube is in a
                                // secure state, not moving and not accepting motion commands. Only after a reset
                                // command which resets this flag, the module will return to the normal operation
                                // mode. An emergency stop can be caused automatically by the module in case of an
                                // error or by the user when sending a Halt command.
#define STATE_POWERFAULT 0x00000008 //This flag defines an error of the servo amplifier. This flag si set in conjuction with
                                    // STATE_ERROR. In most cases the module needs to be switched off to reset this
                                    // error. One of the flags 18 through 23 will be set to explain the cause.
#define STATE_TOW_ERROR 0x00000010 // Tow error: The servo loop was not able to follow the target position within the given
                                   // limit. The maximum tow can be adjsuted using the parameter „MaxDeltaPos". Check
                                   // if the module was overloaded.
#define STATE_COMM_ERROR 0x00000020 // A data transmission error occured. If this error flag is set you should check the
                                    // transmission line (cable, end resistor, grounding).
#define STATE_SWR 0x00000040 // This flag shows the state of the home switch. Flag set means home switch is active,
                             // This is no error flag.
#define STATE_SW1 0x00000080 // This flag shows the state of the Limit switch 1. Flag set means limit switch 1 is active.
                             // This is no error flag.
#define STATE_SW2 0x00000100 // This flag shows the state of the Limit switch 2. Flag set means limit switch 2 is active.
                             // This is no error flag.
#define STATE_BRAKEACTIVE 0x00000200 // This flag shows the state of the brake. Flag set means brake is active and servo loop
                                     // is open. This is no error flag and it is used only if a brake is installed.
#define STATE_CURLIMIT 0x00000400 // This flag is a warning of the servo loop. It has reached the maximum current output.
                                     // The drive is working at its limits. This flag can be reset by the Reset command. It is
                                     // no error flag
#define STATE_MOTION 0x00000800 // This flag indicates the drive is in motion. It is set and reset automatically.
#define STATE_RAMP_ACC 0x00001000 // This flag indicates the drive is in acceleration when controlled by ramp motion
                                  // commands. It is automatically reset when the ramp motion profile has ended.
#define STATE_RAMP_STEADY 0x00002000 // This flag indicates the drive is moving at constant speed when controlled by ramp
                                     // motion commands. It is automatically reset when the ramp motion profile has ended.
#define STATE_RAMP_DEC 0x00004000 // This flag indicates the drive is in deceleration when controlled by ramp motion
                                  // commands. It is automatically reset when the ramp motion profile has ended.
#define STATE_RAMP_END 0x00008000 // This flag indicates the end of a ramp motion profile. The drive is not moving.
#define STATE_INPROGRESS 0x00010000 // This flag is only used in Step motion control. It indicates a Step motion command is
                                       // in progress.
#define STATE_FULLBUFFER 0x00020000 // This flag is only used in Step motion control. It indicates a Step motion command
                                    // was pushed to the command stack. This happens when the module receives a Step
                                    // motion command while STATE_INPROGRESS is set. Upon completion of the
                                    // currently executed step command, the buffered one will automatically be executed.
#define STATE_POW_VOLT_ERR 0x00040000 // This flag is set in conjunction with STATE_POWERFAULT. It indicates the a voltage
                                      // drop in the servo loop. This error can be reset after the normal voltage level has
                                      // been restored. Check your power supply.
#define STATE_POW_FET_TEMP 0x00080000 // This flag is set in conjunction with STATE_POWERFAULT. The power transistors
                                      // have overheated and the servo loop has been disabled. Power must be switched of
                                      // to reset this error. It is due to overload or too high ambient temperature.
#define STATE_POW_WDG_TEMP 0x00100000 // This flag is set in conjunction with STATE_POWERFAULT. The motor has overheated
                                      // and the servo loop has been disabled. Power must be switched of to reset this
                                      // error. It is due to overload or too high ambient temperature.
#define STATE_POW_SHORTCUR 0x00200000 //This flag is set in conjunction with STATE_POWERFAULT. A short curcuit occured.
                                      //  The servo loop has been disabled. The power must be switched of to reset this error.
                                      //  The module has been overlaoded. If this error cannot be reset consult your service
                                      //  partner.
#define STATE_POW_HALLERR 0x00400000 // This flag is set in conjunction with STATE_POWERFAULT. An error occured in reading
                                     // the hall effect sensors of the motor. The motor has been overheated. Power
                                     // must be switched off to reset this error.
#define STATE_POW_INTEGRALERR 0x00800000 // This flag is set in conjunction with STATE_POWERFAULT. The drive has been
                                         // overloaded and the servo loop has been disabled. Power must be switched off to
                                         // reset this error. Check your apllication and the load situations of the drive.
#define STATE_CPU_OVERLOAD 0x01000000 // Communication breakdown between CPU and current controller. Power must be
                                         // switched off. Please consult your service partner.
#define STATE_BEYOND_HARD 0x02000000 // This flag indicates the module has reached the hard limit. An emergency stop has
                                        // been executed automatically. To remove the module from this position you need to
                                        // follow the procedure described in "Plustronik™ Operation System: Disorder".
#define STATE_BEYOND_SOFT 0x04000000 // This flag indicates the module has reached the soft limit. An emergency stop has
                                     // been executed automatically. This flag can be reset by a Reset command.

#define STATE_POW_SETUP_ERR 0x08000000 // Error in initializing the current controller. Module settings disaccord with controller
                                       // configuration (5A/10A types). Power must be switched off. Please consult your
                                       // service partner. Available from version 3.5.14

struct Cmd
{
    int id;
    std::vector<unsigned char> data;

    Cmd( int id, std::vector<unsigned char> data) :
        id(id),
        data(data)
    {}
};

class PCubeProtocol {
public:

    static std::vector<unsigned char> makeData(
        int commandId,
        int motionId,
        int val);

    static std::vector<unsigned char> makeData(
        int commandId,
        int motionId,
        float val);

    static std::vector<unsigned char> makeData(
        int commandId,
        int motionId,
        float x,
        int y);

    static std::vector<unsigned char> makeData(
        int commandId,
        int motionId,
        int x,
        int y);

    static std::vector<unsigned char> makeData(
        int commandId,
        int motionId);

    static std::vector<unsigned char> makeData(unsigned char x);

    static float toFloat(
        unsigned char b0,
        unsigned char b1,
        unsigned char b2,
        unsigned char b3 )
    {
        ToData tofloat(1);
        tofloat.data[0] = b0;
        tofloat.data[1] = b1;
        tofloat.data[2] = b2;
        tofloat.data[3] = b3;
        return tofloat.float_val;
    };

    static int toInt(
        unsigned char b0,
        unsigned char b1,
        unsigned char b2,
        unsigned char b3 )
    {
        ToData toint(1);
        toint.data[0] = b0;
        toint.data[1] = b1;
        toint.data[2] = b2;
        toint.data[3] = b3;
        return toint.int_val;
    };

private:
    union ToData
    {
        ToData(float val) : float_val(val) {}
        ToData(int val) : int_val(val) {}

        int int_val;
        float float_val;
        unsigned char data[4];
    };
};

#endif /*PCUBEPROTOCOL_HPP_*/
