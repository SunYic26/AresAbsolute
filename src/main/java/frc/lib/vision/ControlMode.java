// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.vision;

import java.util.HashMap;
import java.util.Map;

/**
*
*/
public class ControlMode {

    public enum LedMode {
        pipeline(0),   //0	use the LED Mode set in the current pipeline
        forceOff(1),   //1	force off
        forceBlink(2), //2	force blink
        forceOn(3);    //3	force on 

        private static final Map<Double, LedMode> MY_MAP = new HashMap<Double, LedMode>();

        static {
            for (LedMode LedMode : values()) {
            MY_MAP.put(LedMode.getValue(), LedMode);
            }
        }

        private double value;

        private LedMode(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static LedMode getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }

}


    public enum CamMode {
        vision(0),
        driver(1);

        private static final Map<Double, CamMode> MY_MAP = new HashMap<Double, CamMode>();

        static {
            for (CamMode CamMode : values()) {
            MY_MAP.put(CamMode.getValue(), CamMode);
            }
        }

        private double value;

        private CamMode(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static CamMode getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }
    }

    public enum StreamType {
        Standard(0),
        PiPMain(1),
        PiPSecondary(2);

        private static final Map<Double,  StreamType> MY_MAP = new HashMap<Double,  StreamType>();

        static {
            for ( StreamType  StreamType : values()) {
            MY_MAP.put( StreamType.getValue(),  StreamType);
            }
        }

        private double value;

        private  StreamType(double value) {
            this.value = value;
        }

        public double getValue() {
            return value;
        }

        public static  StreamType getByValue(double value) {
            return MY_MAP.get(value);
        }

        public String toString() {
            return name();
        }

    }

    public enum  Snapshot {

        on(1), off(0);

        private static final Map<Double,  Snapshot> MY_MAP = new HashMap<Double,  Snapshot>();

        static {
          for ( Snapshot  Snapshot : values()) {
            MY_MAP.put( Snapshot.getValue(),  Snapshot);
          }
        }

        private double value;

        private  Snapshot(double value) {
          this.value = value;
        }

        public double getValue() {
          return value;
        }

        public static  Snapshot getByValue(double value) {
          return MY_MAP.get(value);
        }

        public String toString() {
          return name();
        }

      }

      public enum  Advanced_Target {

        one(0), two(1), three(2);

        private static final Map<Integer,  Advanced_Target> MY_MAP = new HashMap<Integer,  Advanced_Target>();

        static {
          for ( Advanced_Target  Advanced_Target : values()) {
            MY_MAP.put( Advanced_Target.getValue(),  Advanced_Target);
          }
        }

        private Integer value;

        private  Advanced_Target(Integer value) {
          this.value = value;
        }

        public Integer getValue() {
          return value;
        }

        public static  Advanced_Target getByValue(Integer value) {
          return MY_MAP.get(value);
        }

        public String toString() {
          return name();
        }

      }

      public enum  Advanced_Crosshair {

        one(0), two(1);

        private static final Map<Integer,  Advanced_Crosshair> MY_MAP = new HashMap<Integer,  Advanced_Crosshair>();

        static {
          for ( Advanced_Crosshair  Advanced_Crosshair : values()) {
            MY_MAP.put( Advanced_Crosshair.getValue(),  Advanced_Crosshair);
          }
        }

        private Integer value;

        private  Advanced_Crosshair(Integer value) {
          this.value = value;
        }

        public Integer getValue() {
          return value;
        }

        public static  Advanced_Crosshair getByValue(Integer value) {
          return MY_MAP.get(value);
        }

        public String toString() {
          return name();
        }

      }
}