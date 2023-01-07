#include "CowConstants.h"

#include "CowLib/CowLib.h"

#include <iostream>
#include <iterator>
#include <regex>
#include <cstring>

CowConstants *CowConstants::m_SingletonInstance = NULL;

const static std::string REGEX_COMMENT       = "#[^\\n]*";
const static std::string REGEX_LEFT_BRACKET  = "\\[";
const static std::string REGEX_RIGHT_BRACKET = "\\]";
const static std::string REGEX_EQUALS        = "=";
const static std::string REGEX_SEMICOLON     = ";";
const static std::string REGEX_VALUE         = "-?[0-9]*\\.?[0-9]+";
const static std::string REGEX_KEY           = "[A-Za-z0-9_-]+";
const static std::string REGEX_TOKENIZER     = "(\\[|\\]|#[^\\n]*|=|;|-?[0-9]*\\.?[0-9]+|[A-Za-z0-9_-]+)";

const static std::regex COMMENT{ REGEX_COMMENT };
const static std::regex LEFT_BRACKET{ REGEX_LEFT_BRACKET };
const static std::regex RIGHT_BRACKET{ REGEX_RIGHT_BRACKET };
const static std::regex EQUALS{ REGEX_EQUALS };
const static std::regex SEMICOLON{ REGEX_SEMICOLON };
const static std::regex VALUE{ REGEX_VALUE };
const static std::regex KEY{ REGEX_KEY };

CowConstants *CowConstants::GetInstance()
{
    if (m_SingletonInstance == NULL)
    {
        m_SingletonInstance = new CowConstants();
    }
    return m_SingletonInstance;
}

CowConstants::CowConstants()
{
    RestoreData();
}

// Returns a bool indicating if the key/value pair exists
bool CowConstants::DoesKeyExist(std::string key)
{
    return (bool) m_Data.count(key);
}

double CowConstants::GetValueForKey(const char *key)
{
    if (DoesKeyExist(std::string(key)))
    {
        return m_Data[std::string(key)].numeric;
    }
    else
    {
        printf("Missing constant: %s!!\nKilling FRC_RobotTask\n", key);
        CowLib::PrintToLCD("Missing constant!!\n%s\n\nKilling FRC_RobotTask", key);
        exit(1); // Kill the robot
        return 0;
    }
}

void CowConstants::SetValueForKey(std::string key, std::string value)
{
    m_Data[key].value   = value;
    m_Data[key].numeric = atof(value.c_str());
}

void CowConstants::GrammarError(const char *expectedTokenDescription, std::string value, std::string receivedToken)
{
    printf("Error: expected %s before \"%s\", instead got \"%s\"\nAborting parsing\n", expectedTokenDescription, value.c_str(), receivedToken.c_str());
}

void CowConstants::RestoreData(const char *filename)
{
    // Clear constants info
    m_Data.clear();

    // Load in our file
    std::string data;
    std::ifstream file(filename, std::ios::in | std::ios::binary);
    if (!file)
    {
        printf("Error: could not read %s\nNo constants were loaded\n", filename);
        return;
    }

    // Read all of the file into data
    file.seekg(0, std::ios::end);
    data.resize(file.tellg());
    file.seekg(0, std::ios::beg);
    file.read(&data[0], data.size());
    file.close();

    // Regex parsing
    ParseINI(data);
}

CowConstants::CowConstantType CowConstants::GetConstantType(const std::string &token)
{
    // Sorted in order of most commonly seen pattern
    if (std::regex_match(token, VALUE))
    {
        return CONSTANT_TYPE_NUMBER;
    }
    else if (std::regex_match(token, KEY))
    {
        return CONSTANT_TYPE_NAME;
    }
    else if (std::regex_match(token, EQUALS))
    {
        return CONSTANT_TYPE_EQUALS;
    }
    else if (std::regex_match(token, SEMICOLON))
    {
        return CONSTANT_TYPE_SEMICOLON;
    }
    else if (std::regex_match(token, LEFT_BRACKET))
    {
        return CONSTANT_TYPE_LEFT_BRACKET;
    }
    else if (std::regex_match(token, RIGHT_BRACKET))
    {
        return CONSTANT_TYPE_RIGHT_BRACKET;
    }
    if (std::regex_match(token, COMMENT))
    {
        return CONSTANT_TYPE_COMMENT;
    }
    else
    {
        // This should never happen but just in case
        return CONSTANT_TYPE_INVALID;
    }
}

void CowConstants::Tokenize(const std::string &data, std::vector<CowConstants::CowConstantToken> &tokens)
{
    std::regex r{ REGEX_TOKENIZER };

    // Tokenize the entire file
    std::sregex_token_iterator it{ std::begin(data), std::end(data), r };
    std::sregex_token_iterator reg_end;

    for (; it != reg_end; ++it)
    {
        CowConstants::CowConstantToken token;
        token.type = GetConstantType(*it);

        // Any comment matches get ignored and will not be added to tokenized list
        if (token.type == CONSTANT_TYPE_COMMENT)
        {
            continue;
        }

        token.value = *it;
        tokens.push_back(token);
    }
}

void CowConstants::ParseINI(const std::string &data)
{
    char robotName[256] = { 0 };
    gethostname(robotName, 256);

    printf("Hostname: %s\r\n", robotName);

    // Token container
    std::vector<CowConstants::CowConstantToken> tokens;
    Tokenize(data, tokens);

    char *currentSection = NULL;

    // Start parsing INI grammar
    if (tokens.size() == 0)
    {
        // This is a really bizzare case, not sure how to write a descriptive error
        printf("Warning: Invalid constants file\nNo constants were loaded\n");
        return;
    }

    // Add an end of file token so that we don't miss dangling tokens
    CowConstants::CowConstantToken eof;
    eof.type  = CONSTANT_TYPE_EOF;
    eof.value = "<EOF>";
    tokens.push_back(eof);

    // Make sure our first token is okay before we loop
    if (tokens[0].type != CONSTANT_TYPE_LEFT_BRACKET && tokens[0].type != CONSTANT_TYPE_NAME)
    {
        return GrammarError("\"[\" or name", tokens[1].value, tokens[0].value);
    }

    // This loop looks at every token after the first one and makes sure it makes sense to go after the token behind it
    // For example, putting a semicolon after a left bracket wouldn't make since, and causes an error
    for (unsigned int i = 1; i < tokens.size(); i++)
    {
        // CowLib::CowLexer::st_Token t = tokens[i];
        CowConstants::CowConstantToken t = tokens[i];
        if (t.type == CONSTANT_TYPE_LEFT_BRACKET && tokens[i - 1].type != CONSTANT_TYPE_RIGHT_BRACKET && tokens[i - 1].type != CONSTANT_TYPE_SEMICOLON)
        {
            return GrammarError("\"]\" or \";\"", t.value, tokens[i - 1].value);
        }
        else if (t.type == CONSTANT_TYPE_NAME && tokens[i - 1].type != CONSTANT_TYPE_LEFT_BRACKET && tokens[i - 1].type != CONSTANT_TYPE_RIGHT_BRACKET
                 && tokens[i - 1].type != CONSTANT_TYPE_SEMICOLON)
        {
            return GrammarError("\"]\" or \";\"", t.value, tokens[i - 1].value);
        }
        else if (t.type == CONSTANT_TYPE_RIGHT_BRACKET)
        {
            if (tokens[i - 1].type != CONSTANT_TYPE_NAME)
            {
                return GrammarError("name", t.value, tokens[i - 1].value);
            }
            else if (i == 1)
            {
                printf("Error: unmatched \"]\" after \"%s\"\nAborting parsing\n", tokens[i - 1].value.c_str());
                return;
            }
            else if (tokens[i - 2].type != CONSTANT_TYPE_LEFT_BRACKET)
            {
                printf("Error: unmatched \"]\" after \"%s\"\nAborting parsing\n", tokens[i - 1].value.c_str());
                return;
            }
            else
            {
                currentSection = (char *) tokens[i - 1].value.c_str();
            }
        }
        else if (t.type == CONSTANT_TYPE_EQUALS)
        {
            if (tokens[i - 1].type != CONSTANT_TYPE_NAME)
            {
                return GrammarError("name", t.value, tokens[i - 1].value);
            }
        }
        else if (t.type == CONSTANT_TYPE_NUMBER)
        {
            if (tokens[i - 1].type != CONSTANT_TYPE_EQUALS)
            {
                return GrammarError("\"=\"", t.value, tokens[i - 1].value);
            }
            else
            {
                if (currentSection == NULL || strcmp(currentSection, robotName) == 0)
                {
                    if (currentSection != NULL)
                    {
                        // printf("%s.", currentSection);
                    }
                    // printf("%s: %s\n", tokens[i-2].value.c_str(), t.value.c_str());
                    SetValueForKey(tokens[i - 2].value, t.value);
                }
            }
        }
        else if (t.type == CONSTANT_TYPE_SEMICOLON)
        {
            if (tokens[i - 1].type != CONSTANT_TYPE_NUMBER)
            {
                return GrammarError("number", t.value, tokens[i - 1].value);
            }
        }
        else if (t.type == CONSTANT_TYPE_EOF)
        {
            if (tokens[i - 1].type != CONSTANT_TYPE_RIGHT_BRACKET && tokens[i - 1].type != CONSTANT_TYPE_SEMICOLON)
            {
                return GrammarError("\"]\" or \";\"", t.value, tokens[i - 1].value);
            }
        }
    }
}
