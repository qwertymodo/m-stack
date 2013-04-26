/*****************************
 USB Header File
 Alan Ott
 3-12-2008
*****************************/

#ifndef USB_H_
#define USB_H_

/** @file usb.h
 *  @brief Signal 11 USB Stack
 *  @defgroup public_api Public API
 */

/** @addtogroup public_api
 *  @{
 */

#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>

#include "usb_config.h"

#ifdef __XC16__
#pragma pack(push, 1)
#elif __XC8
#else
#error "Compiler not supported"
#endif

/** @defgroup ch9_items USB Chapter 9 Enumerations and Descriptors
 *  @brief Packet structs from Chapter 9 of the USB spec which deals with
 *  device enumeration.
 *
 *  For more information about these structures, see Chapter 9 of the USB
 *  specification, available from http://www.usb.org .
 *  @addtogroup ch9_items
 *  @{
 */

/** USB PIDs */
enum PID {
	PID_OUT = 0x01,
	PID_IN  = 0x09,
	PID_SOF = 0x05,
	PID_SETUP = 0x0D,
	PID_DATA0 = 0x03,
	PID_DATA1 = 0x0B,
	PID_DATA2 = 0x07,
	PID_MDATA = 0x0F,
	PID_ACK   = 0x02,
	PID_NAK   = 0x0A,
	PID_STALL = 0x0E,
	PID_NYET  = 0x06,
	PID_PRE   = 0x0C,
	PID_ERR   = 0x0C,
	PID_SPLIT = 0x08,
	PID_PING  = 0x04,
	PID_RESERVED = 0x00,
};

/** Destination type
 *
 * This is present in the SETUP packet's bmRequestType field as Direction.
 */
enum DestinationType {
	DEST_DEVICE = 0,
	DEST_INTERFACE = 1,
	DEST_ENDPOINT = 2,
	DEST_OTHER_ELEMENT = 3,
};

/** Request type
 *
 * These are present in the SETUP packet's bmRequestType field as Type.
 */
enum RequestType {
	REQUEST_TYPE_STANDARD = 0,
	REQUEST_TYPE_CLASS    = 1,
	REQUEST_TYPE_VENDOR   = 2,
	REQUEST_TYPE_RESERVED = 3,
};

/** Control Request
 *
 * These are requests sent in the SETUP packet's bRequest field.
 */
enum StandardControlRequest {
	GET_STATUS = 0x0,
	CLEAR_FEATURE = 0x1,
	SET_FEATURE = 0x3,
	SET_ADDRESS = 0x5,
	GET_DESCRIPTOR = 0x6,
	SET_DESCRIPTOR = 0x7,
	GET_CONFIGURATION = 0x8,
	SET_CONFIGURATION = 0x9,
	GET_INTERFACE = 0xA,
	SET_INTERFACE = 0xB,
	SYNCH_FRAME = 0xC,
};

/** Standard Descriptor Types */
enum DescriptorTypes {
	DESC_DEVICE = 0x1,
	DESC_CONFIGURATION = 0x2,
	DESC_STRING = 0x3,
	DESC_INTERFACE = 0x4,
	DESC_ENDPOINT = 0x5,
	DESC_DEVICE_QUALIFIER = 0x6,
	DESC_OTHER_SPEED_CONFIGURATION = 0x7,
	DESC_INTERFACE_POWER = 0x8,
	DESC_OTG = 0x9,
	DESC_DEBUG = 0xA,
	DESC_INTERFACE_ASSOCIATION = 0xB,
};

/** Endpoint Attributes */
enum EndpointAttributes {
	EP_CONTROL = 0x0,
	EP_ISOCHRONOUS = 0x1,
	EP_BULK = 0x2,
	EP_INTERRUPT = 0x3,

	/* More bits here for ISO endpoints only. */
};

/** The SETUP packet, as defined by the USB specification.
 *
 * The contents of the packet sent from the host during the SETUP stage of
 * every control transfer
 */
struct setup_packet {
	union {
		struct {
			uint8_t destination : 5; /**< @see enum DestinationType */
			uint8_t type : 2;        /**< @see enum RequestType */
			uint8_t direction : 1;   /**< 0=out, 1=in */
		};
		uint8_t bmRequestType;
	} REQUEST;
	uint8_t bRequest;  /**< Dependent on @p type. @see enum StandardControlRequest */
	uint16_t wValue;
	uint16_t wIndex;
	uint16_t wLength;
};

/** Device Descriptor */
struct device_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType; /**< set to DESC_DEVICE */
	uint16_t bcdUSB; /**< Set to 0x0200 for USB 2.0 */
	uint8_t bDeviceClass;
	uint8_t bDeviceSubclass;
	uint8_t bDeviceProtocol;
	uint8_t bMaxPacketSize0; /**< Max packet size for ep 0. Must be 8, 16, 32, or 64. */
	uint16_t idVendor;
	uint16_t idProduct;
	uint16_t bcdDevice;
	uint8_t  iManufacturer; /**< index of manufacturer string descriptor */
	uint8_t  iProduct;      /**< index of product string descriptor */
	uint8_t  iSerialNumber; /**< index of serial number string descriptor */
	uint8_t  bNumConfigurations;
};

/** Configuration Descriptor */
struct configuration_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType; /**< Set to DESC_CONFIGURATION */
	uint16_t wTotalLength;
	uint8_t bNumInterfaces;
	uint8_t bConfigurationValue;
	uint8_t iConfiguration; /**< index of string descriptor */
	uint8_t bmAttributes;
	uint8_t bMaxPower; /**< one-half the max power required by this device. */
};

/** Interface Descriptor */
struct interface_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType; /**< Set to DESC_INTERFACE */
	uint8_t bInterfaceNumber;
	uint8_t bAlternateSetting;
	uint8_t bNumEndpoints;
	uint8_t bInterfaceClass;
	uint8_t bInterfaceSubclass;
	uint8_t bInterfaceProtocol;
	uint8_t iInterface;
};

struct hid_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType;
	uint16_t bcdHID;
	uint8_t bCountryCode;
	uint8_t bNumDescriptors;
	uint8_t bDescriptorType2;
	uint16_t wDescriptorLength;
	//bDescriptorType
	//wDescriptorLength
};

/** Endpoint Descriptor */
struct endpoint_descriptor {
	// ...
	uint8_t bLength;
	uint8_t bDescriptorType; /**< Set to DESC_ENDPOINT */
	uint8_t bEndpointAddress;
	uint8_t bmAttributes;
	uint16_t wMaxPacketSize;
	uint8_t bInterval;
};

/** String Descriptor */
struct string_descriptor {
	uint8_t bLength;
	uint8_t bDescriptorType; /**< Set to DESC_STRING */
	uint16_t chars[];
};

/* Doxygen end-of-group for ch9_items */
/** @}*/


/** @defgroup descriptor_items   Descriptor Items
 *  @brief Items defined by the application which are involved in
 *  the enumeration of the device.
 *
 *  The items listed in this section are macro names. An application needs
 *  to define these macro names in usb_config.h to whatever actual C names
 *  are used in the application for these items (typically in
 *  usb_descriptors.c).
 *
 *  It is required that the application #define these items in the
 *  application's @p usb_config.h so the USB stack can retrieve the Chapter
 *  9 descriptors to send to the host.
 *
 *  While this sounds complex, it is not. See the example programs and their
 *  usb_descriptors.c that come with the USB stack for an example of what is
 *  required and how to easily implement it.
 *
 *  @addtogroup descriptor_items
 *  @{
 */


/** String Descriptor Function
 *
 * The USB stack will call this function to retrieve string descriptors from
 * the application. This allows the flexibility for the application to read
 * some strings (like serial numbers) from non-const locations (like EEPROM).
 *
 * @param string_number   The string number requested
 * @param ptr             A pointer to a pointer which should be set to the
 *                        requested string descriptor by this function.
 * @returns
 *   Return the length of the string descriptor in bytes or -1 if the string
 *   requested does not exist.
 */
extern int16_t USB_STRING_DESCRIPTOR_FUNC(uint8_t string_number, const void **ptr);

/** Device Descriptor
 *
 * This is the device's device descriptor as defined by the USB
 * specification, chapter 9.  @p USB_DEVICE_DESCRIPTOR must be defined in
 * usb_config.h to be the name of the device descriptor structure, which
 * will often be located in the application's usb_descriptors.c.
 */
extern const struct device_descriptor USB_DEVICE_DESCRIPTOR;

/** Configuration Descriptor
 *
 * This is an array of the device's configuration descriptors, as defined by
 * the USB specification, chapter 9.  USB_CONFIG_DESCRIPTOR_MAP must be
 * defined to be the name of an array of pointers to
 * configuration_descriptor objects, often in the application's
 * usb_descriptors.c.  The order is not important because the @p
 * bConfigurationValue field is used by the USB stack to determine the
 * configuration number for each configuration descriptor.  It is important
 * that wTotalLength in each configuration descriptor be correct, as this is
 * used by the USB stack to determine the number of bytes to use (It is
 * recommended to use the sizeof() operator for this field).
 *
 * See the example programs that come with the USB stack (specificallyl
 * usb_descriptors.c) for a simple example of what is required.
 */
extern const struct configuration_descriptor *USB_CONFIG_DESCRIPTOR_MAP[];


/* Doxygen end-of-group for descriptor_items */
/** @}*/



 /** @defgroup static_callbacks Static Callbacks
  *  @brief Optional static callback macros to be defined in the
  *  application's usb_config.h.
  *
  *  If desired, #define these callback functions in your application's
  *  @p usb_config.h to receive notification about specific events which
  *  happen during enumeration and otherwise. While these are not strictly
  *  required for all devices, they may be required depending on your
  *  device configuration.
  *
  *  @addtogroup static_callbacks
  *  @{
  */

#ifdef SET_CONFIGURATION_CALLBACK
/** @brief Callback for SET_CONFIGURATION requests
 *
 * SET_CONFIGURATION_CALLBACK() is called whenever a @a SET_CONFIGURATION
 * request is received from the host.  The configuration parameter is the
 * new configuration the host requests.  If configuration is zero, then the
 * device is to enter the @a ADDRESS state.  If it is non-zero then the device
 * is to enter the @a CONFIGURED state.
 *
 * There's no way to reject this request. The host commands a configuration
 * be set, and it shall be done.
 */
void SET_CONFIGURATION_CALLBACK(uint8_t configuration);
#endif

#ifdef GET_DEVICE_STATUS_CALLBACK
/** @brief Callback for GET_STATUS requests
 *
 * GET_DEVICE_STATUS_CALLBACK() is called when a @a GET_STATUS request is
 * received from the host for the device (not the interface or the endpoint).
 * The callback is to return the status of the device as a 16-bit
 * unsigned integer per section 9.4.5 of the USB 2.0 specification.
 *   Bit 0 (LSB) - 0=bus_powered, 1=self_powered
 *   Bit 1       - 0=no_remote_wakeup, 1=remote_wakeup
 *   Bits 2-15   - reserved, set to zero.
 */
uint16_t GET_DEVICE_STATUS_CALLBACK();
#endif

#ifdef ENDPOINT_HALT_CALLBACK
/** @brief Callback for SET_FEATURE or CLEAR_FEATURE with ENDPOINT_HALT
 *
 * ENDPOINT_HALT_CALLBACK() is called when a @a SET_FEATURE or @a
 * CLEAR_FEATURE is received from the host changing the endpoint halt value.
 * This is a notification only.  There is no way to reject this request.
 *
 * @brief endpoint   The endpoint identifier of the affected endpoint
 *                   (direction and number, e.g.: 0x81 means EP 1 IN).
 * @brief halted     1=endpoint_halted (set), 0=endpoint_not_halted (clear)
 */
void ENDPOINT_HALT_CALLBACK(uint8_t endpoint, bool halted);
#endif

#ifdef SET_INTERFACE_CALLBACK
/** @brief Callback for the SET_INTERFACE request
 *
 * SET_INTERFACE_CALLBACK() is called when a @a SET_INTERFACE request is
 * received from the host.  @a SET_INTERFACE is used to set the alternate
 * setting for the specified interface.  The parameters @p interface and @p
 * alt_setting come directly from the device request (from the host).  The
 * callback should return 0 if the new alternate setting can be set or -1 if
 * it cannot.  This callback is completely unnecessary if you only have one
 * alternate setting (alternate setting zero) for each interface.
 *
 * @param interface     The interface on which to set the alternate setting
 * @param alt_setting   The alternate setting
 * @returns
 *   Return 0 for success and -1 for error (will send a STALL to the host)
 */
int8_t SET_INTERFACE_CALLBACK(uint8_t interface, uint8_t alt_setting);
#endif

#ifdef GET_INTERFACE_CALLBACK
/** @brief Callback for the GET_INTERFACE request
 *
 * GET_INTERFACE_CALLBACK() is called when a @a GET_INTERFACE request is
 * received from the host.  @a GET_INTERFACE is a request for the current
 * alternate setting selected for a given interface.  The application should
 * return the interface's current alternate setting from this callback
 * function.  If this callback is not present, zero will be returned as the
 * current alternate setting for all interfaces.
 *
 * @param interface   The interface queried for current altertate setting
 * @returns
 *   Return the current alternate setting for the interface requested or -1
 *   if the interface does not exist.
 */
int8_t GET_INTERFACE_CALLBACK(uint8_t interface);
#endif

#ifdef UNKNOWN_SETUP_REQUEST_CALLBACK
/** @brief Callback for an unrecognized SETUP request
 *
 * UNKNOWN_SETUP_REQUEST_CALLBACK() is called when a SETUP packet is
 * received with a request (bmRequestType,bRequest) which is unknown to the
 * the USB stack.  This could be because it is a vendor-defined request or
 * because it is some other request which is not supported, for example if
 * you were implementing a device class in your application.  There are four
 * ways to handle this:
 *
 * 0. For unknown requests, return -1. This will send a STALL to the host.
 * 1. For requests which have no data stage, the callback should call
 *    @p usb_send_data_stage() with a length of zero to send a zero-length
 *    packet back to the host.
 * 2. For requests which expect an IN data stage, the callback should call
 *    @p usb_send_data_stage() with the data to be sent, and a callback
 *    which will get called when the data stage is complete.  The callback
 *    is required, and the data buffer passed to @p usb_send_data_stage()
 *    must remain valid until the callback is called.
 * 3. For requests which will come with an OUT data stage, the callback
 *    should call @p usb_start_receive_ep0_data_stage() and provide a
 *    buffer and a callback which will get called when the data stage has
 *    completed.  The callback is required, and the data in the buffer
 *    passed to usb_start_receive_ep0_data_stage() is not valid until the
 *    callback is called.
 *
 * It is worth noting that only one control transfer can be active at any
 * given time.  Once UNKNOWN_SETUP_REQUEST_CALLBACK() has been called, it
 * will not be called again until the next transfer, meaning that if the
 * application-provided UNKNOWN_SETUP_REQUEST_CALLBACK() function performs
 * one of options 1-3 above, the callback function passed to @p
 * usb_send_data_stage() or @p usb_start_receive_ep0_data_stage() will be
 * called before UNKNOWN_SETUP_REQUEST_CALLBACK() can be called again.
 * Thus, it is safe to use the same buffer for all control transfers if
 * desired.
 *
 * @param pkt   The SETUP packet
 * @returns
 *   Return 0 if the SETUP can be handled or -1 if it cannot. Returning -1
 *   will cause STALL to be returned to the host.
 */
int8_t UNKNOWN_SETUP_REQUEST_CALLBACK(const struct setup_packet *pkt);
#endif

#ifdef UNKNOWN_GET_DESCRIPTOR_CALLBACK
/** @brief Callback for a GET_DESCRIPTOR request for an unknown descriptor
 *
 * UNKNOWN_GET_DESCRIPTOR_CALLBACK() is called when a @a GET_DESCRIPTOR
 * request is received from the host for a descriptor which is unrecognized
 * by the USB stack.  This could be because it is a vendor-defined
 * descriptor or because it is some other descriptor which is not supported,
 * for example if you were implementing a device class in your application.
 * The callback function should set the @p descriptor pointer and return the
 * number of bytes in the descriptor.  If the descriptor is not supported,
 * the callback should return -1, which will cause a STALL to be sent to the
 * host.
 *
 * @param pkt          The SETUP packet with the request in it.
 * @param descriptor   a pointer to a pointer which should be set to the
 *                     descriptor data.
 * @returns
 *   Return the length of the descriptor pointed to by @p *descriptor, or -1
 *   if the descriptor does not exist.
 */
int16_t UNKNOWN_GET_DESCRIPTOR_CALLBACK(const struct setup_packet *pkt, const void **descriptor);
#endif

/* Doxygen end-of-group for static_callbacks */
/** @}*/

/** @cond INTERNAL */

//TODO Find a better place for this stuff
#define USB_ARRAYLEN(X) (sizeof(X)/sizeof(*X))
#define STATIC_SIZE_CHECK_EQUAL(X,Y) typedef char USB_CONCAT(STATIC_SIZE_CHECK_LINE_,__LINE__) [(X==Y)?1:-1]
#define USB_CONCAT(X,Y)  USB_CONCAT_HIDDEN(X,Y)
#define USB_CONCAT_HIDDEN(X,Y) X ## Y

STATIC_SIZE_CHECK_EQUAL(sizeof(struct endpoint_descriptor), 7);
STATIC_SIZE_CHECK_EQUAL(sizeof(struct hid_descriptor), 9);
STATIC_SIZE_CHECK_EQUAL(sizeof(struct interface_descriptor), 9);
STATIC_SIZE_CHECK_EQUAL(sizeof(struct configuration_descriptor), 9);
STATIC_SIZE_CHECK_EQUAL(sizeof(struct device_descriptor), 18);
STATIC_SIZE_CHECK_EQUAL(sizeof(struct setup_packet), 8);


/** @endcond */

/** @brief Initialize the USB library and hardware
 *
 * Call this function at the beginning of execution. This function initializes
 * the USB peripheral hardware and software library. After calling this
 * funciton, the library will handle enumeration automatically when attached
 * to a host.
 */
void usb_init(void);

/** @brief Update the USB library and hardware
 *
 * This function services the USB peripheral's interrupts and handles all
 * tasks related to enumeration and transfers. It is non-blocking. Whether an
 * application should call this function depends on the @p USB_USE_INTERRUPTS
 * #define. If @p USB_USE_INTERRUPTS is not defined, this function should be
 * called periodically from the main application. If @p USB_USE_INTERRUPTS is
 * defined, it should be called from interrupt context. On PIC24, this will
 * happen automatically, as the interrupt handler is embedded in usb.c. On
 * 8-bit PIC since the interrupt handlers are shared, this function will need
 * to be called from the application's interrupt handler.
 */
void usb_service(void);

/** @brief Get the device configuration
 *
 * Get the device configuration as set by the host. If the device is not
 * in the CONFIGURED state, 0 will be returned.
 *
 * @see usb_is_configured()
 * @returns
 *   Return the device configuration or 0 if the device is not configured.
 */
uint8_t usb_get_configuration(void);

/** @brief Determine whether the device is in the Configured state
 *
 * Return whether the device is in the configured state. During enumeration,
 * the device will start at the DEFAULT state, transition through ADDRESS,
 * and eventually reach CONFIGURED.  The host can also command the device
 * out of the configured state (and back into ADDRESS).  The application
 * shouldn't use any of the endpoints unless in the CONFIGURED state.
 *
 * @see usb_get_configuration()
 */
#define usb_is_configured() (usb_get_configuration() != 0)

/** @brief Get a pointer to an endpoint's input buffer
 *
 * This function returns a pointer to an endpoint's input buffer. Call this
 * to get a location to copy IN data to in order to send it to the host.
 * Remember that IN data is data which goes from the device to the host.
 * The maximum length of this buffer is defined by the application in
 * usb_config.h (eg: @p EP_1_IN_LEN).  It is wise to call
 * @p usb_in_endpoint_busy() before calling this function.
 *
 * @param endpoint   The endpoint requested
 * @returns
 *   Return a pointer to the endpoint's buffer.
 */
unsigned char *usb_get_in_buffer(uint8_t endpoint);

/** @brief Send an endpoint's IN buffer to the host
 *
 * Send the data in the IN buffer for the specified endpoint to the host.
 * Since USB is a polled bus, this only queues the data for sending. It will
 * actually be sent when the device receives an IN token for the specified
 * endpoint. To check later whether the data has been sent, call
 * @p usb_in_endpoint_busy(). If the endpoint is busy, a transmission is
 * pending, but has not been actually transmitted yet.
 *
 * @param endpoint   The endpoint on which to send data
 * @param len        The amount of data to send
 */
void usb_send_in_buffer(uint8_t endpoint, size_t len);

/** @brief Check whether an IN endpoint is busy
 *
 * An IN endpoint is said to be busy if there is data in its buffer and it
 * is waiting for an IN token from the host in order to send it (or if it is
 * in the process of sending the data).
 *
 * @param endpoint   The endpoint requested
 * @returns
 *    Return true if the endpoint is busy, or false if it is not.
 */
bool usb_in_endpoint_busy(uint8_t endpoint);

/** @brief Check whether an endpoint is halted
 *
 * Check if an endpoint has been halted by the host. If an endpoint is
 * halted, don't call usb_send_in_buffer().
 *
 * @see ENDPOINT_HALT_CALLBACK.
 *
 * @param endpoint   The endpoint requested
 * @returns
 *   Return true if the endpointed is halted, or false if it is not.
 */
bool usb_in_endpoint_halted(uint8_t endpoint);

/** @brief Check whether an OUT endpoint has received data
 *
 * Check if an OUT endpoint has completed a transaction and has received
 * data from the host.  If it has, the application should call @p
 * usb_get_out_buffer() to get the data and then call @p
 * usb_arm_out_endpoint() to enable reception of the next transaction.
 *
 * @param endpoint   The endpoint requested
 * @returns
 *   Return true if the endpoint has received data, false if it has not.
 */
bool usb_out_endpoint_has_data(uint8_t endpoint);

/** @brief Re-enable reception on an OUT endpoint
 *
 * Re-enable reception on the specified endpoint. Call this function after
 * @p usb_out_endpoint_has_data() indicated that there was data available,
 * and after the application has dealt with the data.  Calling this function
 * gives the specified OUT endpoint's buffer back to the USB stack to
 * receive the next transaction.
 *
 * @param endpoint   The endpoint requested
 */
void usb_arm_out_endpoint(uint8_t endpoint);

/** @brief Check whether an OUT endpoint is halted
 *
 * Check if an endpoint has been halted by the host. If an OUT endpoint is
 * halted, the USB stack will automatically return STALL in response to any
 * OUT tokens.
 *
 * @see ENDPOINT_HALT_CALLBACK.
 *
 * @param endpoint   The endpoint requested
 * @returns
 *   Return true if the endpointed is halted, or false if it is not.
 */
bool usb_out_endpoint_halted(uint8_t endpoint);

/** @brief Get a pointer to an endpoint's OUT buffer
 *
 * Call this function to get a pointer to an endpoint's OUT buffer after
 * @p usb_out_endpoint_has_data() returns @p true (indicating that
 * an OUT transaction has been received). Do not call this function if
 * @p usb_out_endpoint_has_data() does not return true.
 *
 * @param endpoint   The endpoint requested
 * @param buffer     A pointer to a pointer which will be set to the
 *                   endpoint's OUT buffer.
 * @returns
 *   Return the number of bytes received.
 */
uint8_t usb_get_out_buffer(uint8_t endpoint, const unsigned char **buffer);

/** @brief Endpoint 0 data stage callback definition
 *
 * This is the callback function type expected to be passed to @p
 * usb_start_receive_ep0_data_stage() and @p usb_send_data_stage().
 * Callback functions will be called by the stack when the event for which
 * they are registered occurs.
 *
 * @param transfer_ok   @a true if transaction completed successfully, or
 *                      @a false if there was an error
 * @param context       A pointer to application-provided context data
 */
typedef void (*usb_ep0_data_stage_callback)(bool transfer_ok, void *context);

/** @brief Start the data stage of an OUT control transfer
 *
 * Start the data stage of a control transfer for a transfer which has an
 * OUT data stage.  Call this from @p UNKNOWN_SETUP_REQUEST_CALLBACK for OUT
 * control transfers which being handled by the application.  Once the
 * transfer has completed, @p callback will be called with the @p context
 * pointer provided.  The @p buffer should be considered to be owned by the
 * USB stack until the @p callback is called and should not be modified by the
 * application until this time.
 *
 * @see UNKNOWN_SETUP_REQUEST_CALLBACK
 *
 * @param buffer     A buffer in which to place the data
 * @param len        The number of bytes to expect. This must be less than or
 *                   equal to the number of bytes in the buffer, and for
 *                   proper setup packets will be the wLength parameter.
 * @param callback   A callback function to call when the transfer completes.
 *                   This parameter is mandatory. Once the callback is
 *                   called, the transfer is over, and the buffer can be
 *                   considered to be owned by the application again.
 * @param context    A pointer to be passed to the callback.  The USB stack
 *                   does not dereference this pointer
 */
void usb_start_receive_ep0_data_stage(char *buffer, size_t len,
	usb_ep0_data_stage_callback callback, void *context);

/** @brief Start the data stage of an IN control transfer
 *
 * Start the data stage of a control transfer for a transfer which has an IN
 * data stage.  Call this from @p UNKNOWN_SETUP_REQUEST_CALLBACK for IN
 * control transfers which are being handled by the application.  Once the
 * transfer has completed, @p callback will be called with the @p context
 * pointer provided.  The @p buffer should be considered to be owned by the
 * USB stack until the callback is called and should not be modified by the
 * application until this time.  Do not pass in a buffer which is on the
 * stack.  The data will automatically be split into as many transactions as
 * necessary to complete the transfer.
 *
 * @see UNKNOWN_SETUP_REQUEST_CALLBACK
 *
 * @param buffer     A buffer containing the data to send. This should be a
 *                   buffer capable of having an arbitrary lifetime.  Do not
 *                   use a stack variable for this buffer, and do not free
 *                   this buffer until the callback has been called.
 * @param len        The number of bytes to send
 * @param callback   A callback function to call when the transfer completes.
 *                   This parameter is mandatory. Once the callback is
 *                   called, the transfer is over, and the buffer can be
 *                   considered to be owned by the application again.
 * @paramcontext     A pointer to be passed to the callback. The USB stack
 *                   does not dereference this pointer.
 */
void usb_send_data_stage(char *buffer, size_t len,
	usb_ep0_data_stage_callback callback, void *context);


/* Doxygen end-of-group for public_api */
/** @}*/

#ifdef __XC16__
#pragma pack(pop)
#elif __XC8
#else
#error "Compiler not supported"
#endif

#endif /* USB_H_ */
