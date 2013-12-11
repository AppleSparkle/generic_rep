/*
 * wiz820io.h
 *
 *  Created on: 11.11.2013
 *      Author: R2D2
 */

#define COMMON_BASE 0x0000
#define SHAR0				(COMMON_BASE + 0x0009)

#define CH_BASE                         (COMMON_BASE + 0x4000)

#define Sn_CR(ch)			(CH_BASE + ch * CH_SIZE + 0x0001)

#define MR				(COMMON_BASE + 0x0000)
/**
 @brief Gateway IP Register address
 */
#define GAR0			        (COMMON_BASE + 0x0001)
/**
 @brief Subnet mask Register address
 */
#define SUBR0			        (COMMON_BASE + 0x0005)
/**
 @brief Source MAC Register address
 */
#define SHAR0				(COMMON_BASE + 0x0009)
/**
 @brief Source IP Register address
 */
#define SIPR0				(COMMON_BASE + 0x000F)
/**
 @brief Interrupt Register
 */
#define IR				(COMMON_BASE + 0x0015)
/**
 @brief Socket Interrupt Register
 */
#define IR2				(COMMON_BASE + 0x0034)
/**
 @brief PHY Status Register
 */
#define PHY				(COMMON_BASE + 0x0035)
/**
 @brief Interrupt mask register
 */
#define IMR				(COMMON_BASE + 0x0036)
/**
 @brief Socket Interrupt Mask Register
 */
#define IMR2				(COMMON_BASE + 0x0016)
/**
 @brief Timeout register address( 1 is 100us )
 */
#define RTR				(COMMON_BASE + 0x0017)
/**
 @brief Retry count reigster
 */
#define RCR				(COMMON_BASE + 0x0019)
/**
 @brief Authentication type register address in PPPoE mode
 */
#define PATR0			        (COMMON_BASE + 0x001C)
#define PPPALGO                         (COMMON_BASE + 0x001E)
/**
 @briefPPP LCP Request Timer register  in PPPoE mode
 */
#define PTIMER 		                (COMMON_BASE + 0x0028)
/**
 @brief PPP LCP Magic number register  in PPPoE mode
 */
#define PMAGIC 		                (COMMON_BASE + 0x0029)
/**
 @brief chip version register address
 */
#define VERSIONR			(COMMON_BASE + 0x001F)
/**
 @brief Unreachable IP register address in UDP mode
 */
#define UIPR0				(COMMON_BASE + 0x002A)
/**
 @brief Unreachable Port register address in UDP mode
 */
#define UPORT0			        (COMMON_BASE + 0x002E)
/**
 @brief set Interrupt low level timer register address
 */
#define INTLEVEL0			(COMMON_BASE + 0x0030)
#define INTLEVEL1			(COMMON_BASE + 0x0031)
/**
 @brief socket register
*/
#define CH_BASE                         (COMMON_BASE + 0x4000)
/**
 @brief	size of each channel register map
 */
#define CH_SIZE		                0x0100
/**
 @brief socket Mode register
 */
#define Sn_MR(ch)		        (CH_BASE + ch * CH_SIZE + 0x0000)
/**
 @brief channel Sn_CR register
 */
#define Sn_CR(ch)			(CH_BASE + ch * CH_SIZE + 0x0001)
/**
 @brief channel interrupt register
 */
#define Sn_IR(ch)			(CH_BASE + ch * CH_SIZE + 0x0002)
/**
 @brief channel status register
 */
#define Sn_SR(ch)			(CH_BASE + ch * CH_SIZE + 0x0003)
/**
 @brief source port register
 */
#define Sn_PORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0004)
/**
 @brief Peer MAC register address
 */
#define Sn_DHAR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0006)
/**
 @brief Peer IP register address
 */
#define Sn_DIPR0(ch)			(CH_BASE + ch * CH_SIZE + 0x000C)
/**
 @brief Peer port register address
 */
#define Sn_DPORT0(ch)			(CH_BASE + ch * CH_SIZE + 0x0010)
/**
 @brief Maximum Segment Size(Sn_MSSR0) register address
 */
#define Sn_MSSR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0012)
/**
 @brief Protocol of IP Header field register in IP raw mode
 */
#define Sn_PROTO(ch)			(CH_BASE + ch * CH_SIZE + 0x0014)

/**
 @brief IP Type of Service(TOS) Register
 */
#define Sn_TOS(ch)			(CH_BASE + ch * CH_SIZE + 0x0015)
/**
 @brief IP Time to live(TTL) Register
 */
#define Sn_TTL(ch)			(CH_BASE + ch * CH_SIZE + 0x0016)
/**
 @brief Receive memory size reigster
 */
#define Sn_RXMEM_SIZE(ch)	        (CH_BASE + ch * CH_SIZE + 0x001E)
/**
 @brief Transmit memory size reigster
 */
#define Sn_TXMEM_SIZE(ch)	        (CH_BASE + ch * CH_SIZE + 0x001F)
/**
 @brief Transmit free memory size register
 */
#define Sn_TX_FSR0(ch)		        (CH_BASE + ch * CH_SIZE + 0x0020)
/**
 @brief Transmit memory read pointer register address
 */
#define Sn_TX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0022)
/**
 @brief Transmit memory write pointer register address
 */
#define Sn_TX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x0024)
/**
 @brief Received data size register
 */
#define Sn_RX_RSR0(ch)		        (CH_BASE + ch * CH_SIZE + 0x0026)
/**
 @brief Read point of Receive memory
 */
#define Sn_RX_RD0(ch)			(CH_BASE + ch * CH_SIZE + 0x0028)
/**
 @brief Write point of Receive memory
 */
#define Sn_RX_WR0(ch)			(CH_BASE + ch * CH_SIZE + 0x002A)
/**
 @brief socket interrupt mask register
 */
#define Sn_IMR(ch)			(CH_BASE + ch * CH_SIZE + 0x002C)
/**
 @brief frag field value in IP header register
 */
#define Sn_FRAG(ch)			(CH_BASE + ch * CH_SIZE + 0x002D)
/**
 @brief Keep Timer register
 */
#define Sn_KEEP_TIMER(ch)		(CH_BASE + ch * CH_SIZE + 0x002F)

/* MODE register values */
#define MR_RST			0x80 /**< reset */
#define MR_WOL			0x20 /**< Wake on Lan */
#define MR_PB			0x10 /**< ping block */
#define MR_PPPOE		0x08 /**< enable pppoe */
#define MR_LB  		        0x04 /**< little or big endian selector in indirect mode */
#define MR_AI			0x02 /**< auto-increment in indirect mode */
#define MR_IND			0x01 /**< enable indirect mode */

/* IR register values */
#define IR_CONFLICT	        0x80 /**< check ip confict */
#define IR_UNREACH	        0x40 /**< get the destination unreachable message in UDP sending */
#define IR_PPPoE		0x20 /**< get the PPPoE close message */
#define IR_MAGIC		0x10 /**< get the magic packet interrupt */
#define IR_SOCK(ch)	        (0x01 << ch) /**< check socket interrupt */

/* Sn_MR values */
#define Sn_MR_CLOSE		0x00		/**< unused socket */
#define Sn_MR_TCP		0x01		/**< TCP */
#define Sn_MR_UDP		0x02		/**< UDP */
#define Sn_MR_IPRAW	        0x03		/**< IP LAYER RAW SOCK */
#define Sn_MR_MACRAW	        0x04		/**< MAC LAYER RAW SOCK */
#define Sn_MR_PPPOE		0x05		/**< PPPoE */
#define Sn_MR_ND		0x20		/**< No Delayed Ack(TCP) flag */
#define Sn_MR_MULTI		0x80		/**< support multicating */

/* Sn_CR values */
#define Sn_CR_OPEN		0x01		/**< initialize or open socket */
#define Sn_CR_LISTEN		0x02		/**< wait connection request in tcp mode(Server mode) */
#define Sn_CR_CONNECT	        0x04		/**< send connection request in tcp mode(Client mode) */
#define Sn_CR_DISCON		0x08		/**< send closing reqeuset in tcp mode */
#define Sn_CR_CLOSE		0x10		/**< close socket */
#define Sn_CR_SEND		0x20		/**< update txbuf pointer, send data */
#define Sn_CR_SEND_MAC	        0x21		/**< send data with MAC address, so without ARP process */
#define Sn_CR_SEND_KEEP     	0x22		/**<  send keep alive message */
#define Sn_CR_RECV		0x40		/**< update rxbuf pointer, recv data */

#define Sn_IR_SEND_OK			0x10		/**< complete sending */
#define Sn_IR_TIMEOUT			0x08		/**< assert timeout */
#define Sn_IR_RECV			0x04		/**< receiving data */
#define Sn_IR_DISCON			0x02		/**< closed socket */
#define Sn_IR_CON			0x01		/**< established connection */

/* Sn_SR values */
#define SOCK_CLOSED			0x00		/**< closed */
#define SOCK_INIT 			0x13		/**< init state */
#define SOCK_LISTEN			0x14		/**< listen state */
#define SOCK_SYNSENT	   		0x15		/**< connection state */
#define SOCK_SYNRECV		   	0x16		/**< connection state */
#define SOCK_ESTABLISHED		0x17		/**< success to connect */
#define SOCK_FIN_WAIT			0x18		/**< closing state */
#define SOCK_CLOSING		   	0x1A		/**< closing state */
#define SOCK_TIME_WAIT			0x1B		/**< closing state */
#define SOCK_CLOSE_WAIT			0x1C		/**< closing state */
#define SOCK_LAST_ACK			0x1D		/**< closing state */
#define SOCK_UDP			0x22		/**< udp socket */
#define SOCK_IPRAW			0x32		/**< ip raw mode socket */
#define SOCK_MACRAW			0x42		/**< mac raw mode socket */
#define SOCK_PPPOE			0x5F		/**< pppoe socket */

/* IP PROTOCOL */
#define IPPROTO_IP              0           /**< Dummy for IP */
#define IPPROTO_ICMP            1           /**< Control message protocol */
#define IPPROTO_IGMP            2           /**< Internet group management protocol */
#define IPPROTO_GGP             3           /**< Gateway^2 (deprecated) */
#define IPPROTO_TCP             6           /**< TCP */
#define IPPROTO_PUP             12          /**< PUP */
#define IPPROTO_UDP             17          /**< UDP */
#define IPPROTO_IDP             22          /**< XNS idp */
#define IPPROTO_ND              77          /**< UNOFFICIAL net disk protocol */
#define IPPROTO_RAW             255         /**< Raw IP packet */

#define __DEF_IINCHIP_MAP_BASE__ 0x0000
#define __DEF_IINCHIP_MAP_TXBUF__ (COMMON_BASE + 0x8000) /* Internal Tx buffer address of the iinchip */
#define __DEF_IINCHIP_MAP_RXBUF__ (COMMON_BASE + 0xC000) /* Internal Rx buffer address of the iinchip */

#define MAX_SOCK_NUM 8

static unsigned char I_STATUS[MAX_SOCK_NUM];
static unsigned int SMASK[MAX_SOCK_NUM]; /**< Variable for Tx buffer MASK in each channel */
static unsigned int RMASK[MAX_SOCK_NUM]; /**< Variable for Rx buffer MASK in each channel */
static unsigned int SSIZE[MAX_SOCK_NUM]; /**< Max Tx buffer size by each channel */
static unsigned int RSIZE[MAX_SOCK_NUM]; /**< Max Rx buffer size by each channel */
static unsigned int SBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Tx buffer base address by each channel */
static unsigned int RBUFBASEADDRESS[MAX_SOCK_NUM]; /**< Rx buffer base address by each channel */
static unsigned char SUBNET[4];

void Reset_W5200(void);
void SET_MAC(void);
unsigned char LISTEN_TCP (unsigned char s);
void close(unsigned char s);
void ESTABLISH_SOCKET(unsigned char s, unsigned char protocol,
							   unsigned int port, unsigned char flag);


void WIZNET_INIT( unsigned char * tx_size, unsigned char * rx_size	);
void SETRETRANSMISSION(unsigned int timeout, unsigned char retry);
void SETSOURCEIP(void);
void SETGATEWAY(void);

void Delay_us( unsigned char time_us );
void Delay_ms( unsigned int time_ms );



unsigned int wiz_read_buf(unsigned int addr, unsigned char * buf, unsigned int len);
unsigned char incr_windowfull_retry_cnt(unsigned char s);
void init_windowfull_retry_cnt(unsigned char s);
unsigned char getISR(unsigned char s);
void putISR(unsigned char s, unsigned char val);
unsigned int getIINCHIP_RxMAX(unsigned char s);
unsigned int getIINCHIP_TxMAX(unsigned char s);
unsigned int getIINCHIP_RxMASK(unsigned char s);
unsigned int getIINCHIP_TxMASK(unsigned char s);
unsigned int getIINCHIP_RxBASE(unsigned char s);
unsigned int getIINCHIP_TxBASE(unsigned char s);


void send_data_processing(unsigned char s, unsigned char *data, unsigned int len);
void write_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst, unsigned int len);
unsigned int wiz_write_buf(unsigned int addr,unsigned char* buf,unsigned int len);
unsigned int getSn_TX_FSR(unsigned char s);
unsigned int send(unsigned char s, /*const*/ unsigned char * buf, unsigned int len, unsigned char retry);
void read_data(unsigned char s, volatile unsigned char * src, volatile unsigned char * dst,
		unsigned int len);
void recv_data_processing(unsigned char s, unsigned char *data, unsigned int len);
unsigned int WNET_RECIEVE(unsigned char s, unsigned char * buf, unsigned int len);
unsigned int getSn_RX_RSR(unsigned char s);
void disconnect(unsigned char s);




