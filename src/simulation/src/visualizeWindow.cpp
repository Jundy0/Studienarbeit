#include "visualizeWindow.h"

VisualizeWindow::VisualizeWindow(SelfdrivingVehicle *selfdrivingVehicle)
{
    this->selfdrivingVehicle = selfdrivingVehicle;

    this->window = new sf::RenderWindow(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation Visualization", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);

    this->visualizationImage.create(WINDOW_WIDTH, WINDOW_HEIGHT, sf::Color::Transparent);
    this->visualizationTexture.loadFromImage(this->visualizationImage);
    this->visualizationSprite.setTexture(this->visualizationTexture, true);
    this->font.loadFromFile("../res/Arial.ttf");
    this->fpsDisplay.setFont(this->font);
    this->fpsDisplay.setCharacterSize(30);
    this->fpsDisplay.setFillColor(sf::Color::Red);
}

VisualizeWindow::~VisualizeWindow()
{
    delete this->window;
}

void VisualizeWindow::update()
{
    this->pollEvent();

    const float elapsedTime = this->clock.restart().asSeconds();
    this->fps = 1.f / elapsedTime;
}

void VisualizeWindow::render()
{
    window->clear(sf::Color::Yellow);

    // for (size_t i = 0; i < WINDOW_WIDTH; i++)
    // {
    //     for (size_t j = 0; j < WINDOW_HEIGHT; j++)
    //     {
    //         window2Image.setPixel(i, j, sf::Color::Transparent);
    //     }
    // }

    const lidar_point_t *lidarData = this->selfdrivingVehicle->getLidarDataPtr();

    for (size_t i = 0; i < SCAN_COUNT; i++)
    {
        this->visualizationImage.setPixel(lidarData[i].x, lidarData[i].y, sf::Color::Red);
    }

    this->visualizationTexture.loadFromImage(this->visualizationImage);

    this->window->draw(this->visualizationSprite);

    this->fpsDisplay.setString(std::to_string(this->fps));
    this->window->draw(this->fpsDisplay);

    window->display();
}

bool VisualizeWindow::isOpen()
{
    return this->window->isOpen();
}

void VisualizeWindow::close()
{
    this->window->close();
}

void VisualizeWindow::pollEvent()
{
    while (this->window->pollEvent(this->ev))
    {
        switch (this->ev.type)
        {
        case sf::Event::Closed:
            this->window->close();
            break;
        }
    }
}
