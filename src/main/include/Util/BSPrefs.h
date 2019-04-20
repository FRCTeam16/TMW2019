#pragma once

#include <string>
#include <map>

class BSPrefs {
public:
    static BSPrefs* GetInstance();
    double GetDouble(std::string ) const;
    int GetInt(std::string ) const;
    bool GetBool(std::string ) const;

    double GetDouble(std::string , double value) const;
    int GetInt(std::string , int value) const;
    bool GetBool(std::string , bool value) const;

private:
    static BSPrefs* instance;
    BSPrefs();
    std::map<std::string, double> lookupDouble;
    std::map<std::string, int> lookupInt;
    std::map<std::string, bool> lookupBool;
};