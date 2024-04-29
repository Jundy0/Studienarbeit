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

    const Eigen::MatrixXd *gridMap = this->selfdrivingVehicle->getGridMap();
    const Eigen::RowVector2d position = this->selfdrivingVehicle->getPosition();
    const double rot = this->selfdrivingVehicle->getRotation();
    if (gridMap->cols() < 1)
        return;

    sf::Color color;
    size_t gridX, gridY;

    for (size_t x = 0; x < WINDOW_WIDTH; x++)
    {
        for (size_t y = 0; y < WINDOW_HEIGHT; y++)
        {
            gridX = x;
            gridY = WINDOW_HEIGHT - 1 - y;

            if ((*gridMap)(gridY, gridX) >= PROB_OCC)
            {
                color = sf::Color::Black;
            }
            else if ((*gridMap)(gridY, gridX) <= PROB_FREE)
            {
                color = sf::Color::White;
            }
            else
            {
                color = sf::Color::Cyan;
            }
            visualizationImage.setPixel(x, y, color);
        }
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
