#include "visualizeWindow.h"

VisualizeWindow::VisualizeWindow(const std::shared_ptr<SelfdrivingVehicle> &selfdrivingVehicle)
    : selfdrivingVehicle(selfdrivingVehicle)
{
    this->window = std::make_unique<sf::RenderWindow>(sf::VideoMode(WINDOW_WIDTH, WINDOW_HEIGHT), "Simulation Visualization", sf::Style::Titlebar | sf::Style::Close);
    this->window->setFramerateLimit(60);

    this->visualizationImage.create(WINDOW_WIDTH, WINDOW_HEIGHT, sf::Color::Transparent);
    this->visualizationTexture.loadFromImage(this->visualizationImage);
    this->visualizationSprite.setTexture(this->visualizationTexture, true);
    this->visualizationSprite.setScale((float)WINDOW_WIDTH / GRID_WIDTH, (float)WINDOW_HEIGHT / GRID_HEIGHT);

    this->vehicleTexture = sf::Texture();
    this->vehicleTexture.loadFromFile("../res/vehicle.png");
    this->vehicleSprite = sf::Sprite(this->vehicleTexture);
    this->vehicleSprite.setScale((float)WINDOW_WIDTH / MAP_WIDTH, (float)WINDOW_HEIGHT / MAP_HEIGHT);
    const sf::FloatRect pos = this->vehicleSprite.getGlobalBounds();
    this->vehicleSprite.setOrigin(pos.width / 2, pos.height / 2);

    this->font.loadFromFile("../res/Arial.ttf");

    this->fpsDisplay.setFont(this->font);
    this->fpsDisplay.setCharacterSize(30);
    this->fpsDisplay.setFillColor(sf::Color::Red);
}

VisualizeWindow::~VisualizeWindow()
{
}

void VisualizeWindow::update()
{
    this->pollEvent();

    const float elapsedTime = this->clock.restart().asSeconds();
    this->fps = 1.f / elapsedTime;
}

void VisualizeWindow::render()
{
    const Eigen::MatrixXd *gridMap = this->selfdrivingVehicle->getGridMap();
    const Eigen::RowVector2d position = this->selfdrivingVehicle->getPosition();
    const Eigen::RowVector2d destination = this->selfdrivingVehicle->getDestination();
    const double rot = this->selfdrivingVehicle->getRotation();
    const std::vector<Eigen::RowVector2d> path = this->selfdrivingVehicle->getPath();

    sf::Color color;

    for (size_t x = 0; x < GRID_WIDTH; x++)
    {
        for (size_t y = 0; y < GRID_HEIGHT; y++)
        {
            if ((*gridMap)(y, x) >= PROB_OCC)
            {
                color = sf::Color::Black;
            }
            else if ((*gridMap)(y, x) <= PROB_FREE)
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

    for (int i = path.size() - 1; i > 0; i--)
    {
        sf::Vertex line[] =
            {
                sf::Vertex(sf::Vector2f(path[i].x() * (WINDOW_WIDTH / GRID_WIDTH), path[i].y() * (WINDOW_HEIGHT / GRID_HEIGHT)), sf::Color::Red),
                sf::Vertex(sf::Vector2f(path[i - 1].x() * (WINDOW_WIDTH / GRID_WIDTH), path[i - 1].y() * (WINDOW_HEIGHT / GRID_HEIGHT)), sf::Color::Red),
            };

        window->draw(line, 2, sf::Lines);
    }

    const sf::FloatRect vehicleSize = this->vehicleSprite.getGlobalBounds();

    this->vehicleSprite.setPosition(position(0) * WINDOW_WIDTH / MAP_WIDTH, position(1) * WINDOW_HEIGHT / MAP_HEIGHT);
    this->vehicleSprite.setRotation(rot * 180 / M_PI);

    this->window->draw(this->vehicleSprite);

    if (destination != Eigen::RowVector2d(0, 0)) // Default value for Destaination Point
    {
        sf::CircleShape circle = sf::CircleShape(CIRCLE_RADIUS);
        circle.setFillColor(sf::Color::Magenta);
        circle.setPosition(sf::Vector2f((destination.x() * (WINDOW_WIDTH / GRID_WIDTH)), destination.y() * (WINDOW_HEIGHT / GRID_HEIGHT)) - sf::Vector2f(CIRCLE_RADIUS / 2, CIRCLE_RADIUS / 2));
        window->draw(circle);
    }

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
