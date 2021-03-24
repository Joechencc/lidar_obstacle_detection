import org.apache.commons.logging.Log;
import org.ros.message.MessageListener;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Subscriber;

import com.google.common.base.Preconditions;


public class Listener implements NodeMain {

  private Node node;

  @Override
  public void main(Node node) {
    Preconditions.checkState(this.node == null);
    this.node = node;
    try {
      final Log log = node.getLog();
      node.newSubscriber("chatter", "std_msgs/String",
          new MessageListener<org.ros.message.std_msgs.String>() {
            @Override
            public void onNewMessage(org.ros.message.std_msgs.String message) {
              log.info("I heard: \"" + message.data + "\"");
            }
          });
    } catch (Exception e) {
      if (node != null) {
        node.getLog().fatal(e);
      } else {
        e.printStackTrace();
      }
    }
  }

  @Override
  public void shutdown() {
    node.shutdown();
    node = null;
  }

}