#include "../src/Console.hpp"

#include <iostream>
#include <string>

namespace cr = CppReadline;
using ret = cr::Console::ReturnCode;

unsigned info(const std::vector<std::string> &) {
    std::cout << "Welcome to the example console. This command does not really\n"
              << "do anything aside from printing this statement. Thus it does\n"
              << "not need to look into the arguments that are passed to it.\n";
    return ret::Ok;
}

// In this command we implement a basic calculator
unsigned calc(const std::vector<std::string> & input) {
    if ( input.size() != 4 ) {
        // The first element of the input array is always the name of the
        // command as registered in the console.
        std::cout << "Usage: " << input[0] << " num1 operator num2\n";
        // We can return an arbitrary error code, which we can catch later
        // as Console will return it.
        return 1;
    }
    double num1 = std::stod(input[1]),
           num2 = std::stod(input[3]);

    char op = input[2][0];

    double result;
    switch ( op ) {
        case '*':
            result = num1 * num2;
            break;
        case '+':
            result = num1 + num2;
            break;
        case '/':
            result = num1 / num2;
            break;
        case '-':
            result = num1 - num2;
            break;
        default:
            std::cout << "The inserted operator is not supported\n";
            // Again, we can return an arbitrary error code to catch it later.
            return 2;
    }
    std::cout << "Result: " << result << '\n';
    return 0;
}

int main() {
    // We create a console. The '>' character is used as the prompt.
    // Note that multiple Consoles can exist at once, as they automatically
    // manage the underlying global readline state.
    cr::Console c(">");

    // Here we register a new command. The string "info" names the command that
    // the user will have to type in in order to trigger this command (it can
    // be different from the function name).
    c.registerCommand("info", info);
    c.registerCommand("calc", calc);

    // Here we call one of the defaults command of the console, "help". It lists
    // all currently registered commands within the console, so that the user
    // can know which commands are available.
    c.executeCommand("help");

    // Here we try to call a script. The script is read line by line, and each line
    // (be it empty or not) is treated as a separate command. The execution is stopped
    // as soon as any command returns an error.
    c.executeFile("exampleScript");

    // This basic loops continues to read input from the user until a command returns
    // the termination code (ret::Quit). Here it would be one of the default
    // quitting commands ("quit" or "exit").
    /*
     * while ( c.readLine() != ret::Quit );
     */

    // Otherwise we can modify the code to catch Console error codes
    int retCode;
    do {
        retCode = c.readLine();
        // We can also change the prompt based on last return value:
        if ( retCode == ret::Ok )
            c.setGreeting(">");
        else
            c.setGreeting("!>");

        if ( retCode == 1 ) {
            std::cout << "Received error code 1\n";
        }
        else if ( retCode == 2 ) {
            std::cout << "Received error code 2\n";
        }
    }
    while ( retCode != ret::Quit );

    return 0;
}
