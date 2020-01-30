package  org.usfirst.frc.team5975.robot.subsystems;
import  org.usfirst.frc.team5975.robot.subsystems.Pixy2;
import org.usfirst.frc.team5975.robot.subsystems.links.Link;

public class Camera {

	private final Pixy2 pixy;

	public Camera(Link link) {
		pixy = Pixy2.createInstance(link);
		pixy.init();
	}

	public Camera(Link link, int arg) {
		pixy = Pixy2.createInstance(link);
		pixy.init(arg);
	}

	public Pixy2 getPixy() {
		return pixy;
	}

}