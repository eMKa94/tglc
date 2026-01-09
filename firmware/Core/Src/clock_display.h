#pragma once

#ifdef __cplusplus
extern "C" {
#endif

typedef enum displayError_t
{
    DISPLAY_OK = 0,
    DISPLAY_ERROR = -1,
    DISPLAY_ERROR_NULL_POINTER = -2,
    DISPLAY_ERROR_OUT_OF_BOUNDS = -3,
} displayError_t;

displayError_t displayInit(void);


#ifdef __cplusplus
}
#endif
