
typedef unsigned char bool;
typedef unsigned char uint8_t;
typedef unsigned short uint16_t;

typedef struct
{
           uint8_t* buffer    ; ///< buffer pointer
           uint16_t depth     ; ///< max items
           uint16_t item_size ; ///< size of each item
           bool overwritable  ;

  volatile uint16_t count     ; ///< number of items in queue
  volatile uint16_t wr_idx    ; ///< write pointer
  volatile uint16_t rd_idx    ; ///< read pointer

#if defined(CFG_FIFO_MUTEX)
  tu_fifo_mutex_t mutex;
#endif

} tu_fifo_t;

typedef struct
{
    uint8_t role; // device or host
    tu_fifo_t ff;
}osal_queue_def_t;

typedef osal_queue_def_t* osal_queue_t;


bool dcd_edpt_xfer (uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
	__coverity_tainted_data_argument__(buffer);
}

// Taints buffer when used for OUT
bool usbd_edpt_xfer(uint8_t rhport, uint8_t ep_addr, uint8_t * buffer, uint16_t total_bytes)
{
	__coverity_tainted_data_argument__(buffer);
}
/*
void dcd_int_enable (uint8_t rhport) {
	intUnlock();
}

void dcd_int_disable(uint8_t rhport) {
	intLock();
}
*/

static inline bool osal_queue_receive(osal_queue_t const queue_hdl, void* data) {
	__coverity_tainted_data_argument__(data);
}
