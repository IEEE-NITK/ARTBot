#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "artbotsim/artist_frame.h"

class ArtistApp : public QApplication
{
public:
  rclcpp::Node::SharedPtr nh_;

  explicit ArtistApp(int& argc, char** argv)
    : QApplication(argc, argv)
  {
    rclcpp::init(argc, argv);
    nh_ = rclcpp::Node::make_shared("artbotsim");
  }

  ~ArtistApp()
  {
    rclcpp::shutdown();
  }

  int exec()
  {
    artbotsim::ArtistFrame frame(nh_);
    frame.show();

    return QApplication::exec();
  }
};

int main(int argc, char** argv)
{
  ArtistApp app(argc, argv);
  return app.exec();
}