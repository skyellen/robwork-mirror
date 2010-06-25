
/**
   @brief RobWork library.

   All classes and interfaces in this group only have dependencies on boost and
   STL. It is considered the core of the RobWork project.
 */
namespace rwhw {
    /**
     * @brief Camera drivers
     */
    namespace camera {}
}

/**

@defgroup rwhw rwhw
@{
    @copydoc rwhw

    @defgroup camera camera
    @{
        @brief Camera driver wrappers.
    @}

    @defgroup can can
    @{
        @brief CAN bus driver wrappers.
    @}

    @defgroup crsA465 crsA465
    @{
        @brief driver wrapper for the crsA465 robot.
    @}

    @defgroup Fanuc Fanuc
    @{
        @brief Interface for communicating with Fanuc controller through proprietary software
        from Gibotech A/S.
    @}

    @defgroup katana katana
    @{
        @brief driver wrapper for katana robot.
    @}

    @defgroup MotomanIA20 MotomanIA20
    @{
        @brief Driver for motoman IA20
    @}

    @defgroup dockwelder dockwelder
    @{
        @brief Driver interface for the dockwelder robot based on ethernet.
    @}

    @defgroup PowerCube PowerCube
    @{
        @brief Driver interface through RS232 and CAN bus. Works for eqrlier versions of the
        PowerCube interface.
    @}

    @defgroup sdh sdh
    @{
        @brief Wrapper for SDH the 3-finger dextrous hand from schunk.
    @}

    @defgroup serialport serialport
    @{
        @brief multi platform serial port interface. Rather simplistic but usefull for most stuff.
    @}

    @defgroup sick sick
    @{
        @brief driver interface for sick scanner
    @}

    @defgroup swissranger swissranger
    @{
        @brief Driver wrapper for SwissRanger (early version)
    @}

    @defgroup tactile tactile
    @{
        @brief Driver wrapper for Weiss tactile array sensors (early version only)
    @}

    @defgroup
    @{
        @brief Ignore this group. The group has been added to fix a glitch in
        the Doxygen output.
    @}
@}

*/
