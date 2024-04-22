#include <gui/graph_screen/GraphView.hpp>

GraphView::GraphView() :
    tickCounter(0)
{

}

static uint32_t randomish(int32_t seed)
{
    static const uint16_t RAND_MAX_TGFX = 65535;
    static uint32_t last = 0;
    const uint32_t num = (seed * (1337 + last)) % RAND_MAX_TGFX;
    last = num;
    return num;
}

void GraphView::handleTickEvent()
{
    static const uint8_t sine[] =
    {
        46, 47, 48, 49, 50, 51, 52, 53, 54, 56, 57, 58, 59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70,
        71, 72, 73, 74, 74, 75, 76, 77, 78, 79, 79, 80, 81, 81, 82, 83, 83, 84, 85, 85, 86, 86, 87, 87,
        88, 88, 88, 89, 89, 89, 90, 90, 90, 90, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 91, 90, 90,
        90, 90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 80, 80, 79, 78,
        77, 77, 76, 75, 74, 73, 72, 71, 70, 69, 68, 68, 67, 66, 65, 63, 62, 61, 60, 59, 58, 57, 56, 55,
        54, 53, 52, 51, 49, 48, 47, 46, 45, 44, 43, 42, 40, 39, 38, 37, 36, 35, 34, 33, 32, 31, 30, 29,
        28, 26, 25, 24, 23, 23, 22, 21, 20, 19, 18, 17, 16, 15, 14, 14, 13, 12, 11, 11, 10, 9, 9, 8, 7, 7,
        6, 6, 5, 5, 4, 4, 3, 3, 2, 2, 2, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1, 1, 2, 2, 2, 3,
        3, 3, 4, 4, 5, 5, 6, 6, 7, 8, 8, 9, 10, 10, 11, 12, 12, 13, 14, 15, 16, 17, 17, 18, 19, 20, 21, 22,
        23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35, 37, 38, 39, 40, 41, 42, 43, 44,
    };

    tickCounter++;

    if (tickCounter % 2 == 0)
    {
        const uint16_t point = static_cast<int16_t>(sine[static_cast<int16_t>(tickCounter * 0.2f) % 255] + 1 + randomish(tickCounter) % 8);
        const uint16_t insertedIndex = dynamicGraph1.addDataPoint(point);

        if (dynamicGraph1.indexToDataPointXAsInt(insertedIndex) == 1000)
        {
            dynamicGraph1MajorXAxisLabel.setInterval(25);
            dynamicGraph1MajorXAxisLabel.invalidate();
        }
        else if (dynamicGraph1.indexToDataPointXAsInt(insertedIndex) == 10000)
        {
            dynamicGraph1MajorXAxisLabel.setInterval(50);
            dynamicGraph1MajorXAxisLabel.invalidate();
        }
    }
}

void GraphView::handleKeyEvent(uint8_t key)
{
    if (key == '5')
    {
        // Select
        goToScreen1();
    }
}
