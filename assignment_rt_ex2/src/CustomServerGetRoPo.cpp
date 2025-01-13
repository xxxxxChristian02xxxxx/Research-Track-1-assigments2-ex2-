// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <inttypes.h>
#include <memory>
#include "service_msgs/srv/get_robot_position.hpp"
#include "rclcpp/rclcpp.hpp"

using GetRobotPosition = service_msgs::srv::GetRobotPosition;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class GetRobotPositionServer : public rclcpp::Node
{
public:
  GetRobotPositionServer()
  : Node("get_robot_position_server")
  {
    service_ = this->create_service<service_msgs::srv::GetRobotPosition>(
      "/get_robot_position", std::bind(&GetRobotPositionServer::handle_service, this, _1, _2, _3));
  }

private:
    // qui sono sempre lato server, in sto modo, sto usando il server che si trova in un nodo a se stante,
    // il quale semplicemente vede ciò che ha nella request e lo raddoppia per la response quando viene chiamato
    // il clinet è nel nodo robot_moving.cpp con la funzione call_get_robot_position_service



  void handle_service(
  const std::shared_ptr<rmw_request_id_t> request_header,
  const std::shared_ptr<service_msgs::srv::GetRobotPosition::Request> request,
  const std::shared_ptr<service_msgs::srv::GetRobotPosition::Response> response)
  {
  (void)request_header;
    response->pose.position.x = 2*request->x;
    response->pose.position.y = 2*request->y;

 RCLCPP_INFO(
    this->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->x, request->y);





/*   RCLCPP_INFO(
    this->get_logger(),
    "request: %" PRId64 " + %" PRId64, request->a, request->b);
  response->sum = request->a + request->b; */
}
  rclcpp::Service<service_msgs::srv::GetRobotPosition>::SharedPtr service_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GetRobotPositionServer>());
  rclcpp::shutdown();
  return 0;
}