#pragma once
#include <windows.h>

//params handling
typedef void * (*PFN_ED_COCKPIT_GET_PARAMETER_HANDLE        )  (const char * name);
typedef void   (*PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_STRING)  (void * handle, const char * string_value);
typedef void   (*PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_NUMBER)  (void * handle, double number_value);
typedef bool   (*PFN_ED_COCKPIT_PARAMETER_VALUE_TO_NUMBER   )  (const void * handle, double & res, bool interpolated);
typedef bool   (*PFN_ED_COCKPIT_PARAMETER_VALUE_TO_STRING   )  (const void * handle, char * buffer, unsigned buffer_size);
typedef int    (*PFN_ED_COCKPIT_COMPARE_PARAMETERS          )  (void * handle_1, void * handle_2);
 
struct cockpit_param_api
{
    PFN_ED_COCKPIT_GET_PARAMETER_HANDLE             get_parameter_handle;      
    PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_STRING     update_parameter_with_string;
    PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_NUMBER     update_parameter_with_number;
    PFN_ED_COCKPIT_PARAMETER_VALUE_TO_NUMBER        parameter_value_to_number;  
    PFN_ED_COCKPIT_PARAMETER_VALUE_TO_STRING        parameter_value_to_string;
    PFN_ED_COCKPIT_COMPARE_PARAMETERS               compare_parameters;
};
 
inline cockpit_param_api  ed_get_cockpit_param_api()
{
    HMODULE cockpit_dll                 = GetModuleHandleA("CockpitBase.dll"); //assume that we work inside same process
 
    cockpit_param_api ret;
    ret.get_parameter_handle            = (PFN_ED_COCKPIT_GET_PARAMETER_HANDLE)        GetProcAddress(cockpit_dll,"ed_cockpit_get_parameter_handle");
    ret.update_parameter_with_number    = (PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_NUMBER)GetProcAddress(cockpit_dll,"ed_cockpit_update_parameter_with_number");
    ret.update_parameter_with_string    = (PFN_ED_COCKPIT_UPDATE_PARAMETER_WITH_STRING)GetProcAddress(cockpit_dll,"ed_cockpit_update_parameter_with_string");
    ret.parameter_value_to_number       = (PFN_ED_COCKPIT_PARAMETER_VALUE_TO_NUMBER)   GetProcAddress(cockpit_dll,"ed_cockpit_parameter_value_to_number");
    ret.parameter_value_to_string       = (PFN_ED_COCKPIT_PARAMETER_VALUE_TO_STRING)   GetProcAddress(cockpit_dll,"ed_cockpit_parameter_value_to_string");  
    ret.compare_parameters              = (PFN_ED_COCKPIT_COMPARE_PARAMETERS)          GetProcAddress(cockpit_dll, "ed_cockpit_compare_parameters");
 
    return ret;
}