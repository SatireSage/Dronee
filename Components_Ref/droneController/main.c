#include "drone_controller.h"

int main()
{
    // Open and configure the port
    configPort();

    char *controllerPacket = malloc(Controller_Buffer_Size);

    sleepForMs(1);

    while (true)
    {
        controllerPacket = createPacket();
        // check if the packet is not Center
        if (strcmp(controllerPacket, "Center") != 0)
        {
            sendPacket(controllerPacket);
        }
        sleepForMs(100);
        if (check_finish(readButton()))
            break;
    }

    free(controllerPacket);

    return 0;
}