#pragma once

#include <string>

using namespace std;

/* ------------------------------------ ANSI COLOR CODES ------------------------------------ */

// Color code
#define BLACK_CODE "\033[0;30m"
#define RED_CODE "\033[0;31m"
#define GREEN_CODE "\033[0;32m"
#define YELLOW_CODE "\033[0;33m"
#define BLUE_CODE "\033[0;34m"
#define MAGENTA_CODE "\033[0;35m"
#define CYAN_CODE "\033[0;36m"
#define WHITE_CODE "\033[0;37m"

// Bold versions
#define BOLD_BLACK_CODE "\033[1;30m"
#define BOLD_RED_CODE "\033[1;31m"
#define BOLD_GREEN_CODE "\033[1;32m"
#define BOLD_YELLOW_CODE "\033[1;33m"
#define BOLD_BLUE_CODE "\033[1;34m"
#define BOLD_MAGENTA_CODE "\033[1;35m"
#define BOLD_CYAN_CODE "\033[1;36m"
#define BOLD_WHITE_CODE "\033[1;37m"

// Background colors
#define BLACK_BG_CODE "\033[0;40m"
#define RED_BG_CODE "\033[0;41m"
#define GREEN_BG_CODE "\033[0;42m"
#define YELLOW_BG_CODE "\033[0;43m"
#define BLUE_BG_CODE "\033[0;44m"
#define MAGENTA_BG_CODE "\033[0;45m"
#define CYAN_BG_CODE "\033[0;46m"
#define WHITE_BG_CODE "\033[0;47m"

// Reset code
#define RESET_CODE "\033[0m"

// C printf style string formatter
inline string format(const char *str, ...)
{
    char buf[1024];
    va_list args;
    va_start(args, str);
    vsprintf(buf, str, args);
    va_end(args);
    return string(buf);
}

// 按指定精度将 vector 输出为 string "[1.2, 3.4]" 形式
template<typename T>
string vec2str(const vector<T>& vec, int precision = 2) {
    ostringstream oss;
    oss << fixed << setprecision(precision);

    oss << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        oss << vec[i];
        if (i < vec.size() - 1) {
            oss << ", ";
        }
    }
    oss << "]";

    return oss.str();
}

// 将 str 包在上下边框中打印出来, 返回输入的字符串本身
inline string prettyPrint(const string &str, const string &title = "", const string &colorCode = "", int borderLength = 70, char borderChar = '=')
{
    int headerHalfLength = floor((borderLength - title.length() - 2) / 2);
    if (headerHalfLength < 0)
        headerHalfLength = 0;
    string header = string(headerHalfLength, borderChar);
    string footer = string(borderLength, borderChar);
    cout << colorCode
         << header << " " << title << " " << header << "\n\n"
         << str << "\n\n"
         << footer 
         << RESET_CODE << "\n" << endl;
    return str;
}

// 将 str 包在一段醒目的标为 ERROR 的红色区块中打印出来, 返回输入的字符串本身
inline string prtErr(const string &str)
{
    return prettyPrint(str, "ERROR", RED_CODE);
}

// 将 str 包在一段醒目的标为 DEBUG 的区块中打印出来, 返回输入的字符串本身
inline string prtDebug(const string &str, string color = CYAN_CODE)
{
    return prettyPrint(str, "DEBUG", color);
}

// 将 str 包在一段醒目的标为 DEBUG 的区块中打印出来, 返回输入的字符串本身
inline string prtWarn(const string &str)
{
    return prettyPrint(str, "WARN", YELLOW_CODE);
}