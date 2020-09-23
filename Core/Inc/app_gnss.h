#ifndef __APP_GNSS
#define __APP_GNSS
#include "bc20.h"
#define GNSS_WORD_LEN 20 //GNSS关键字

typedef struct
{
  uint8_t step;
  bool to_stop;
  bool stopped;
}stu_gnss_read;

typedef struct
{
  uint8_t step;
  bool to_stop;
  bool stopped;
}stu_gnss_upload;

extern stu_gnss_read gnss_read;
extern stu_gnss_upload gnss_upload;
extern char gnss_keyword[GNSS_WORD_LEN];
extern int gnss_keyword_len;
extern char utc_str[GNSS_WORD_LEN];
extern char la_str[GNSS_WORD_LEN];
extern char lo_str[GNSS_WORD_LEN];
extern char sp_str[GNSS_WORD_LEN];
extern char ag_str[GNSS_WORD_LEN];
extern char dir_str[GNSS_WORD_LEN];
extern char la_dir_str[GNSS_WORD_LEN];
extern char lo_dir_str[GNSS_WORD_LEN];
extern char state_str[GNSS_WORD_LEN];
extern char star_num_str[GNSS_WORD_LEN];
extern char hdop_str[GNSS_WORD_LEN];
extern char height_str[GNSS_WORD_LEN];
extern char pa_str[GNSS_WORD_LEN];
extern char utc_date_str[GNSS_WORD_LEN];
extern char dir_dir_str[GNSS_WORD_LEN];
extern char utc_str[GNSS_WORD_LEN];
extern char la_str[GNSS_WORD_LEN];
extern char lo_str[GNSS_WORD_LEN];
extern char sp_str[GNSS_WORD_LEN];
extern char ag_str[GNSS_WORD_LEN];
extern char dir_str[GNSS_WORD_LEN];
extern bool gnss_data_new;
#endif
