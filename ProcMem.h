#ifndef PROCMEM_H
#define PROCMEM_H

#include <Windows.h>
#include <TlHelp32.h>
#include <vector>
#include <string>
#include <stdexcept>

class ProcMem {
public:
    ProcMem();

    bool Process(const wchar_t* processName);
    DWORD Module(const wchar_t* moduleName);
    DWORD GetProcessID() const;

    template <typename T>
    T Read(DWORD address) {
        T value;
        SIZE_T bytesRead;
        if (!ReadProcessMemory(hProcess, reinterpret_cast<LPCVOID>(address), &value, sizeof(T), &bytesRead) || bytesRead != sizeof(T)) {
            throw std::runtime_error("Failed to read memory.");
        }
        return value;
    }

    template <typename T>
    bool Write(DWORD address, T value) {
        SIZE_T bytesWritten;
        if (!WriteProcessMemory(hProcess, reinterpret_cast<LPVOID>(address), &value, sizeof(T), &bytesWritten) || bytesWritten != sizeof(T)) {
            throw std::runtime_error("Failed to write memory.");
            return false;
        }
        return true;
    }

private:
    DWORD processID;
    HANDLE hProcess;

    DWORD GetModuleBaseAddress(DWORD processID, const wchar_t* moduleName);
};

#endif // PROCMEM_H
