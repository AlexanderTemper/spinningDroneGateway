// import libraries
import processing.serial.*;
import java.util.ArrayList;

String serialPortName = "/dev/ttyUSB0";
Serial serialPort; // Serial port object

// Communication
ArrayList<Integer> inputBuffer = new ArrayList<Integer>(); 
int inputBytes = 0;
int lastChar = 0;
boolean start = false;

class Sensor {
  float[] rawValues= {0, 0, 0};
  Graph g;
  float min, max;
  int[] visible = {1, 1, 1};
  int[] scale = {1, 1, 1};
  float[][] diagrammValues = new float[3][1000];
  int posX, posY = 0;
  color[] colors = new color[3];
  float[] xLabels;


  Sensor(int posX, int posY, int w, int h, float min, float max, String label, color[] colors, float[] xLabels) {
    this.posX = posX;
    this.posY = posY;
    this.min = min;
    this.max = max;
    this.g = new Graph(this.posX+90, this.posY+20, w, h, color (20, 20, 200));
    this.g.xLabel=" Samples ";
    this.g.yLabel=label;
    this.g.Title="";  
    this.g.xDiv=20;  
    this.g.xMax=0; 
    this.g.xMin=-100;  
    this.g.yMax=this.max; 
    this.g.yMin=this.min;
    this.colors[0] = colors[0];
    this.colors[1] = colors[1];
    this.colors[2] = colors[2];
    this.xLabels = xLabels;
  }

  void updateDiagramm(float[] raw) {
    for (int i=0; i<3; i++) {
      this.rawValues[i] = raw[i];
      if (i<this.diagrammValues.length) {
        for (int k=0; k<this.diagrammValues[i].length-1; k++) {
          this.diagrammValues[i][k] = this.diagrammValues[i][k+1];
        }
        this.diagrammValues[i][this.diagrammValues[i].length-1] = raw[i] * this.scale[i];
      }
    }
  }
  void draw() {
    fill(this.colors[0]);
    text(nf(this.rawValues[0], 1, 3), this.posX+45, this.posY+20); 
    fill(this.colors[1]);
    text(nf(this.rawValues[1], 1, 3), this.posX+45, this.posY+32); 
    fill(this.colors[2]);
    text(nf(this.rawValues[2], 1, 3), this.posX+45, this.posY+44); 
    this.g.DrawAxis();
    for (int i=0; i<this.diagrammValues.length; i++) {
      this.g.GraphColor = this.colors[i];
      if (this.visible[i]==1)
        this.g.LineGraph(xLabels, this.diagrammValues[i]);
    }
  }
}

Sensor tof;
float[] tofRaw = {0, 0, 0};
PFont f;

public void settings() {
  System.setProperty("jogl.disable.openglcore", "true");
  size(1680, 860, P3D);
}


void setup() {

  ///set line graph colors
  color[] graphColors = new color[3];
  graphColors[0] = color(0, 0, 200);
  graphColors[1] = color(232, 158, 12);
  graphColors[2] = color(255, 0, 0);

  // build x axis values for the line graph
  float[] xLabel = new float[1000];
  for (int k=0; k<xLabel.length; k++) {
    xLabel[k] = k;
  }

  tof = new Sensor(0, 0, width-400, height-50, -500, 500, "Tof", graphColors, xLabel);
  surface.setTitle("filter");

  serialPort = new Serial(this, serialPortName, 115200);
  serialPort.clear();
}

void draw() {
  processSerial();
  background(255);
  tof.draw();
}



public static byte[] intToBytes(int l) {
  byte[] result = new byte[4];
  for (int i = 3; i >= 0; i--) {
    result[i] = (byte)(l & 0xFF);
    l >>= 8;
  }
  return result;
}

void processSerial() {
  while (serialPort.available() > 0) {
    String inBuffer =  serialPort.readStringUntil(10);   
    if (inBuffer != null) {
      println(inBuffer);
      String[] result = inBuffer.split(",");
      if (result.length ==3) {
        int raw = parseInt(result[0].replaceAll("\\D+", "")); //remove non-digits
        int filtered = parseInt(result[1].replaceAll("\\D+", "")); //remove non-digits
        int filtered2 = parseInt(result[2].replaceAll("\\D+", "")); //remove non-digits
        tofRaw[0] = float(raw);
        tofRaw[1] = float(filtered);
        tofRaw[2] = float(filtered2);
        tof.updateDiagramm(tofRaw);
      }
    }
  }
}
