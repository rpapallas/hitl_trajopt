#ifndef COMMONFUNCS
#define COMMONFUNCS

#include <sensor_msgs/JointState.h>

void saveScreenshotToFile(const std::string &filename, int windowWidth, int windowHeight) {
    const int numberOfPixels = windowWidth * windowHeight * 3;
    unsigned char pixels[numberOfPixels];

    glPixelStorei(GL_PACK_ALIGNMENT, 1);
    glReadBuffer(GL_FRONT);
    glReadPixels(0, 0, windowWidth, windowHeight, GL_BGR_EXT, GL_UNSIGNED_BYTE, pixels);

    FILE *outputFile = fopen(filename.c_str(), "w");
    short header[] = {0, 2, 0, 0, 0, 0, (short) windowWidth, (short) windowHeight, 24};

    fwrite(&header, sizeof(header), 1, outputFile);
    fwrite(pixels, numberOfPixels, 1, outputFile);
    fclose(outputFile);
}

vector<string> getObjectNamesFromNumberOfObjects(int numberOfObjects) {
    vector<string> objectNames;
    for (int i = 0; i < numberOfObjects; ++i) {
        string objectName = "object_" + std::to_string(i + 1);
        objectNames.push_back(objectName);
    }

    return objectNames;
}

#endif
