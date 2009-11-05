/****************************************************************************
**	constant and macro
*****************************************************************************/

#ifndef RWHW_ICANAPI_HPP
#define RWHW_ICANAPI_HPP

#include <windows.h>

#ifndef UINT8
#define UINT8	unsigned char
#endif

#ifndef UINT16
#define UINT16	unsigned short
#endif

#ifndef UINT32
#define UINT32	unsigned long
#endif

#ifndef INT32
#define INT32	signed long
#endif

/*
** Values for the CAN node
*/
#define CAN_11B		0		// 11 bit identifier mode
#define CAN_29B		1		// 29 bit identifier mode
#define CAN_TX_QUE	0		// transmit queue
#define CAN_RX_QUE	1		// transmit queue

/*
** VCI return codes of the fucntion
*/
#define	CAN_ERR			0
#define CAN_QUE_EMPTY	0
#define CAN_QUE_FULL	0
#define CAN_OLD			0
#define CAN_OK			1	// successful completion
#define CAN_ERR_HWSW	-1	// function could not be performed
                            // because of hard or software errors

#define CAN_ERR_SUPP	-2	// function is not supported
                            // this way (support-error)

#define CAN_ERR_PARA	-3	// calling parameter(s) is (are)
                            // not correct or out of range

#define CAN_ERR_RES		-4	// resource error
                            // the resource limit exceeded
                            // at creation of a queue or soming else

#define CAN_ERR_QUE		-5	// Queue overrun
                            // One or more objects couldn't be
                            // iinserted into the queue and wew lost.

#define CAN_ERR_TX		-6	// A CAN message couldn't be sent
                            // for a long time
                            // cable error, wrong baud rate etc.

/*
**  Baudrates from CAN in Automation  (CIA)
*/

#define BAUD_10K     0x31,0x1C
#define BAUD_20K     0x18,0x1C
#define BAUD_50K     0x09,0x1C
#define BAUD_100K    0x04,0x1C
#define BAUD_125K    0x03,0x1C
#define BAUD_250K    0x01,0x1C
#define BAUD_500K    0x00,0x1C
#define BAUD_800K    0x00,0x16
#define BAUD_1M		 0x00,0x14


/*****************************************************************
**	data_types
*****************************************************************/
/*
** Informations, delivered from the interface board
*/

typedef struct{
    UINT16	sw_version;
    UINT32	dwAddr;				// local dwAddress
    UINT32  dwBus;				// on which bus
    UINT32  dwSlot;				// which slot
    char    cName[32];          // Name of card.
    UINT16	irq_num;
} CAN_BOARD_INFO;

/*
** CAN-Controller information
*/
typedef struct {
    UINT8			bt0;		// Value set for Baudrate (bustiming reg 0 )
    UINT8			bt1;		// Value set for Baudrate (bustiming reg 1 )
    UINT32			acc_code;	// Value set for acceptance filter (id code)
    UINT32			acc_mask;	// Value set for acceptance filter (relevance mask )
} CAN_INFO;

/*
**	CAN status
*/
typedef struct {
    bool  bus_status;
    bool  error_status;
    bool  tx_status;
    bool  rx_status;
    bool  tx_complete_status;
    bool  tx_buffer_status;
    bool  data_overrun_status;
    bool  rx_buffer_status;
} CAN_STS;

/*
**	Structure of a CAN object
*/
typedef struct {
    UINT32	time_stamp;			// Timestamp for receive queue objects
    UINT32	id;					// Identifier 11/29-Bit
    UINT8	len;				// number of data bytes ( 0~8)
    UINT8	rtr;				// RTR-Bit: 0=dataframe, 1=Remoteframe
    UINT8	a_data[8];			// Array for up to 8 data bytes
} CAN_MSG;


/*
 * @brief Enumerate CANbus interface cards installed on the system
 * @return the number of CANbus cards on the system
 */
typedef INT32 (__stdcall *CAN_CountCardsType)();
extern CAN_CountCardsType CAN_CountCards;

/*
 * @brief Get the card's information
 * @param card_idx [in] card index (0..n)
 * @param p_info [in] pointer to the CAN_BOARD_INFO
 * @return above 0 if CAN_OK else CAN_ERR_PARA
 */
typedef INT32 (__stdcall *CAN_ReadBoardInfoType)(UINT16 card_idx,CAN_BOARD_INFO* p_info);
extern CAN_ReadBoardInfoType CAN_ReadBoardInfo;

/////// CAN Level Functions ///////////////////////////////////////////////////////////////////////////

/**
 * @breif Return the operation mode of the CANbus port
 * @param card_id [in] card index (0..n)
 * @param can_no [in] CAN port index (0..1)
 * @return 0 or above if operation mode (CAN_11B or CAN_29B)
 *
 * @note They are 2 operation mode supported by the ICAN-02 interface board.
 * In “Standard” mode it is possible to transmit and receive standard frame
 * messages only ( 11-bit identifier).
 * In “Extended” mode it is possible to transmit and receive standard frame
 * messages with ( 29-bit identifier), this is what called CAN 2.0B.
 */
typedef INT32 (__stdcall *CAN_ModeType)(UINT16 card_idx, UINT8 can_no);
extern CAN_ModeType CAN_Mode;


/**
 * @brief initialize the CANbus port, when begin to use the CAN port
 * normally, you should execute the function first.
 * @param card_idx [in]: card index (0..n)
 * @param can_no [in]: CAN port index (0..1)
 * @param bt0 [in]: bus timing 0
 * @param bt1 [in]: bus timing 1
 * @param mode [in]: (CAN_11B/ CAN_29B)
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 */
typedef INT32 (__stdcall *CAN_InitType)(UINT16 card_idx, UINT8 can_no,UINT8 bt0, UINT8 bt1, UINT8 mode );
extern CAN_InitType CAN_Init;

/**
 * @brief Reset the CANbus port, after success reset , the port will enter to
 * the reset mode, if you want to start again, you shoud call
 * CAN_Start () again.
 * @param card_idx [in]: card index (0..n)
 * @param can_no [in]: CAN port index (0..1)
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 */
typedef INT32 (__stdcall *CAN_ResetType)(UINT16 card_idx, UINT8 can_no );
extern CAN_ResetType CAN_Reset;

/**
 * @brief Read the CANbus port information
 * @param card_idx (in): card index (0..n)
 * @param can_no (in): CANbus port index (0..1)
 * @param p_info (out): point to the CAN_INFO structure
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 *
 * @note you should pass the CAN_INFO structure (defined in icanapi.h)
 * pointer to this function, if read success , the structure will
 * filled with the data
 */
typedef INT32 (__stdcall *CAN_ReadCanInfoType)(UINT16 card_idx, UINT8 can_no, CAN_INFO *p_info);
extern CAN_ReadCanInfoType CAN_ReadCanInfo;

/**
 * @brief Get the CANbus port status
 * @param card_idx (in): card index (0..n).
 * @param can_no (in): CANbus port index (0..1).
 * @param p_sts (out): point to the structure CAN_STS which include the Return
 * status data.
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 *
 * @note you should pass the CAN_STS structure (defined in icanapi.h)
 * pointer to this function, if read success , the structure will filled with the data
 */
typedef INT32 (__stdcall *CAN_ReadCanStatusType)(UINT16 card_idx, UINT8 can_no, CAN_STS *p_sts);
extern CAN_ReadCanStatusType CAN_ReadCanStatus;

/**
 * @brief Start the CAN port (turn to operation mode )
 *
 * @param card_idx(in): card index (0..n)
 * @param can_no(in): CANbus port index (0..1)
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 */
typedef INT32 (__stdcall *CAN_StartType)(UINT16 card_idx, UINT8 can_no );
extern CAN_StartType CAN_Start;

/**
 * @brief determine the CANbus port running status
 * @param card_idx(in): card index (0..n)
 * @param can_no(in): CANbus port index (0..1)
 * @return true when running false otherwise
 */
typedef bool (__stdcall *CAN_IsRunType)(UINT16 card_idx, UINT8 can_no);
extern CAN_IsRunType CAN_IsRun;

//////// Message Level Functoins ///////////////////////////////////////////////////////////////////////

/**
 * @brief Configure the CAN channel TX/RX buffer (Queue)
 * Incoming messages are placed in a queue in the driver.
 * In most cases the hardware does message buffering as well.
 * @param card_idx (in): card index (0..n)
 * @param can_no (in): CANbus port index (0..n)
 * @param que_type (in): CAN_TX_QUE / CAN_RX_QUE
 * @param que_size (in): the capacity of the receive queue (how many
 * message can placed into queue)
 * @return if above 0 CAN_OK else CAN_ERR_PARA, CAN_ERR
 */
typedef UINT16 (__stdcall *CAN_ConfigQueueType)(UINT16 card_idx, UINT8 can_no,UINT8 que_type, UINT16 que_size);
extern CAN_ConfigQueueType CAN_ConfigQueue;

/**
 * @brief Set the acceptance & mask code
 * @param card_idx(in): card index (0..n)
 * @param can_no(in): CANbus port index (0..1)
 * @param acc_code(in): acceptance code
 * @param acc_mask(in): acceptance mask
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 * @note You can set filters to reduce the number of received messages. ICANLIB supports
 * setting of the hardware filters on the CAN interface board. You can set an
 * acceptance code and acceptance mask which together determine which CAN
 * identifiers are accepted or rejected.
 */
typedef INT32 (__stdcall *CAN_SetAccMaskType)(UINT16 card_idx, UINT8 can_no,UINT32 acc_code, UINT32 acc_mask );
extern CAN_SetAccMaskType CAN_SetAccMask;

/**
 * @brief Queue TX / RX message on queue.
 * You can use this function first to query the messages on the queue
 * before you got those messages use CAN_ReadMsg()
 * @param card_idx (in): card index (0..n).
 * @param can_no (in): CANbus port index (0..1).
 * @param que_type (in): CAN_TX_QUE / CAN_RX_QUE
 * @return int i: if above 0 i is number of messages recieved, if 0 then no
 * messages was recieved else if i is below 0 then CAN_ERR_PARA
 */
typedef INT32 (__stdcall *CAN_CountMsgsType)(UINT16 card_idx, UINT8 can_no,UINT8 que_type);
extern CAN_CountMsgsType CAN_CountMsgs;

/**
 * @brief Send the message with the message ID and data, if the connection
 * is available , the message will send immediately, but if connection
 * is unavailable then the send data will buffer into TX queue and will
 * be re-send after the HW available.
 * @param card_idx(in): card index (0..n).
 * @param can_no(in): CANbus port index (0..1).
 * @param id(in): Message ID
 * @param len(in): the length of data sent.
 * @param p_data(in): point to the data which will be send.
 */
typedef INT32 (__stdcall *CAN_SendMsgType)(UINT16 card_idx, UINT8 can_no, UINT32 id, UINT8 len,UINT8* p_data);
extern CAN_SendMsgType CAN_SendMsg;


typedef INT32 (__stdcall *CAN_ReadMsgType)(UINT16 card_idx, UINT8 can_no, UINT16 count, CAN_MSG* p_obj);
extern CAN_ReadMsgType CAN_ReadMsg;

typedef INT32 (__stdcall *CAN_RequestMsgType)(UINT16 card_idx, UINT8 can_no, UINT32 id, UINT8 len);
extern CAN_RequestMsgType CAN_RequestMsg;

////////// Miscelleous /////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Update the buffer data, which may retrive by Remote Transmission
 * Request Message.
 * @param card_idx(in): card index (0..n).
 * @param can_no(in): CANbus port index (0..1).
 * @param len(in): length of the data updated.
 * @param p_data(in): point to the data which will update.
 * @return if above 0 CAN_OK else CAN_ERR_PARA
 *
 */
typedef INT32 (__stdcall *CAN_UpdateBufObjType)(UINT16 card_idx, UINT8 can_no, UINT8 len, UINT8 *p_data);
extern CAN_UpdateBufObjType CAN_UpdateBufObj;

typedef INT32 (__stdcall *CAN_SelfTestType)(UINT16 card_idx, UINT8 can_no);
extern CAN_SelfTestType CAN_SelfTest;

bool isIEICAN02LibOpen();
bool openIEICAN02Library();
void closeIEICAN02Library();

#endif // end include guard
