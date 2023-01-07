//==================================================
// Copyright (C) 2018 Team 1538 / The Holy Cows
//==================================================

#ifndef __COW_CONSTANTS_H__
#define __COW_CONSTANTS_H__

#include "Declarations.h"

#include <fstream>
#include <map>
#include <string>
#include <unistd.h>
#include <vector>

// Short way to access a constant. There's probably a better way to do this.
#define CONSTANT CowConstants::GetInstance()->GetValueForKey

class CowConstants
{
private:
    // Converting from a std::string to a double is expensive, and almost
    // every constant will be a double, so we cast at load time instead of
    // continuously
    typedef struct
    {
        std::string value;
        double numeric;
    } CowConstant;

    typedef enum
    {
        CONSTANT_TYPE_COMMENT,
        CONSTANT_TYPE_LEFT_BRACKET,
        CONSTANT_TYPE_RIGHT_BRACKET,
        CONSTANT_TYPE_EQUALS,
        CONSTANT_TYPE_SEMICOLON,
        CONSTANT_TYPE_NUMBER,
        CONSTANT_TYPE_NAME,
        CONSTANT_TYPE_EOF,
        CONSTANT_TYPE_INVALID,
    } CowConstantType;

    typedef struct
    {
        // std::string type;
        CowConstantType type;
        std::string value;
    } CowConstantToken;

    std::map<std::string, CowConstant> m_Data;
    static CowConstants *m_SingletonInstance;

    void GrammarError(const char *expectedTokenDescription, std::string value, std::string receivedToken);
    void ParseINI(const std::string &data);
    CowConstants::CowConstantType GetConstantType(const std::string &token);
    void Tokenize(const std::string &data, std::vector<CowConstants::CowConstantToken> &tokens);

    CowConstants();

public:
    void RestoreData(const char *filename = COWCONSTANTS_DEFAULT_FILE);
    static CowConstants *GetInstance();
    double GetValueForKey(const char *key);
    void SetValueForKey(std::string key, std::string value);
    bool DoesKeyExist(std::string key);
};

#endif /* __COW_CONSTANTS_H__ */
