/*
 * Compile with `gcc -o glfwtest glfwtest.c -lglfw`
 * If you are on Linux and get linker errors, you may also need to link with `-lm -ldl -lGL` for some reason
 *
 * Make sure you have the GLFW development libraries installed (`glfw` on arch, `libglfw3-dev` on Debian/Ubuntu, etc.)
 */

#include <stdio.h>
#include <GLFW/glfw3.h>

int main(void) {
    if (!glfwInit()) {
        fprintf(stderr, "Failed to initialize GLFW\n");
        return 1;
    }

    int jid;
    const char* guid = NULL;
    for (jid = GLFW_JOYSTICK_1; jid <= GLFW_JOYSTICK_LAST; ++jid) {
        if (glfwJoystickPresent(jid)) {
            guid = glfwGetJoystickGUID(jid);
            const char* joyName = glfwGetJoystickName(jid);
            if (guid) {
                printf("Joystick %d (%s) GUID: %s\n", jid, joyName, guid);
                // Print if it's a gamepad
                if (glfwJoystickIsGamepad(jid)) {
                    printf("Joystick %d is recognized as a gamepad.\n", jid);
                } else {
                    printf("Joystick %d is not recognized as a gamepad.\n", jid);
                }
            } else {
                printf("Joystick %d (%s) has no GUID\n", jid, joyName);
            }
        }
    }

    if (!guid) {
        printf("No joystick connected.\n");
    }

    glfwTerminate();
    return 0;
}