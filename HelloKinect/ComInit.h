#pragma once
// ComInit.h - small RAII wrapper for COM initialization on a thread.
//
// Uses modern header <combaseapi.h>. Links ole32.lib so callers don't need
// to change project linker settings.
#include <Windows.h>
#include <combaseapi.h>
#pragma comment(lib, "ole32.lib")

struct ComInit {
    HRESULT hr;
    // Default to multithreaded apartment which is generally appropriate for worker threads
    explicit ComInit(DWORD flags = COINIT_MULTITHREADED) : hr(::CoInitializeEx(NULL, flags)) {}
    ~ComInit() {
        if (SUCCEEDED(hr)) {
            ::CoUninitialize();
        }
    }
    bool ok() const { return SUCCEEDED(hr); }
};