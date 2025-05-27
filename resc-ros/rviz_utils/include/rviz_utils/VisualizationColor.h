//
// Created by Zhaohong Liu on 24-9-18.
//

#ifndef RVIZ_UTILS_VISUALIZATIONCOLOR_H
#define RVIZ_UTILS_VISUALIZATIONCOLOR_H

class VisualizationColor {
public:
    // RGB values
    float r, g, b;

    VisualizationColor(float red, float green, float blue) : r(red), g(green), b(blue) {}

    // predefined colors
    static const VisualizationColor RED;
    static const VisualizationColor GREEN;
    static const VisualizationColor BLUE;
    static const VisualizationColor YELLOW;
    static const VisualizationColor CYAN;
    static const VisualizationColor MAGENTA;
    static const VisualizationColor WHITE;
    static const VisualizationColor BLACK;
    static const VisualizationColor PURPLE;
    static const VisualizationColor DARKTEAL;
    static const VisualizationColor ORANGE;
};

#endif //RVIZ_UTILS_VISUALIZATIONCOLOR_H
