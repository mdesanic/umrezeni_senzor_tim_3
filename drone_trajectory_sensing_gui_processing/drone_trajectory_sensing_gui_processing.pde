import processing.serial.*;

Serial myPort;

ArrayList<PVector> points = new ArrayList<PVector>();

int maxPoints = 5000;

boolean demoMode = true; //follows cube edges, disable to read UART
float demoPos = 0.0;

// Cube edge traversal path
PVector[] cubePath = {

  // Bottom square
  new PVector(-100, -100, -100),
  new PVector( 100, -100, -100),
  new PVector( 100,  100, -100),
  new PVector(-100,  100, -100),
  new PVector(-100, -100, -100),

  // Up first vertical
  new PVector(-100, -100,  100),

  // Top square
  new PVector( 100, -100,  100),
  new PVector( 100,  100,  100),
  new PVector(-100,  100,  100),
  new PVector(-100, -100,  100),

  // Remaining verticals
  new PVector( 100, -100,  100),
  new PVector( 100, -100, -100),

  new PVector( 100,  100, -100),
  new PVector( 100,  100,  100),

  new PVector(-100,  100,  100),
  new PVector(-100,  100, -100)
};

void setup() {

  size(1200, 800, P3D);

  println("Available Serial Ports:");

  String[] ports = Serial.list();

  for (int i = 0; i < ports.length; i++) {
    println(i + " : " + ports[i]);
  }

  // Try opening first port if one exists
  try {
    if (ports.length > 0) {
      myPort = new Serial(this, ports[0], 115200);
      myPort.bufferUntil('\n');
      println("Connected to: " + ports[0]);
    } else {
      println("No serial ports found. Running demo mode.");
    }
  }
  catch(Exception e) {
    println("Could not open serial port.");
    println("Running demo mode.");
  }

  smooth(8);
}

void draw() {

  if (demoMode) {
    generateDemoData();
  }

  background(20);

  lights();

  translate(width/2, height/2, 0);

  rotateX(map(mouseY, 0, height, PI, -PI));
  rotateY(map(mouseX, 0, width, -PI, PI));

  drawAxes();
  drawCubeReference();
  drawPath();

  fill(255);
  textSize(16);

  camera();

  if (demoMode) {
    text("DEMO MODE (Cube Edge Test Pattern)", 20, 30);
  } else {
    text("LIVE UART MODE", 20, 30);
  }

  text("Points: " + points.size(), 20, 55);
}

void serialEvent(Serial port) {

  String line = port.readStringUntil('\n');

  if (line == null)
    return;

  line = trim(line);

  String[] parts = split(line, ',');

  if (parts.length != 3)
    return;

  try {

    float x = float(parts[0]);
    float y = float(parts[1]);
    float z = float(parts[2]);

    demoMode = false;

    points.add(new PVector(x, y, z));

    if (points.size() > maxPoints) {
      points.remove(0);
    }
  }
  catch(Exception e) {
    println("Parse error: " + line);
  }
}

void generateDemoData() {

  float speed = 0.03;

  int segmentCount = cubePath.length - 1;

  demoPos += speed;

  if (demoPos >= segmentCount) {
    demoPos = 0;
  }

  int seg = floor(demoPos);

  float t = demoPos - seg;

  PVector a = cubePath[seg];
  PVector b = cubePath[seg + 1];

  PVector p = PVector.lerp(a, b, t);

  points.add(p);

  if (points.size() > maxPoints) {
    points.remove(0);
  }
}

void drawAxes() {

  strokeWeight(3);

  // X axis
  stroke(255, 0, 0);
  line(-300, 0, 0, 300, 0, 0);

  // Y axis
  stroke(0, 255, 0);
  line(0, -300, 0, 0, 300, 0);

  // Z axis
  stroke(0, 120, 255);
  line(0, 0, -300, 0, 0, 300);
}

void drawCubeReference() {

  stroke(80);
  strokeWeight(1);
  noFill();

  pushMatrix();
  box(200);
  popMatrix();
}

void drawPath() {

  if (points.size() < 2)
    return;

  stroke(255, 255, 0);
  strokeWeight(3);
  noFill();

  beginShape();

  for (PVector p : points) {
    vertex(
      p.x,
      -p.y,
      p.z
    );
  }

  endShape();

  // Current point marker
  PVector last = points.get(points.size() - 1);

  pushMatrix();

  translate(
    last.x,
    -last.y,
    last.z
  );

  noStroke();
  fill(255, 0, 255);
  sphere(8);

  popMatrix();
}
