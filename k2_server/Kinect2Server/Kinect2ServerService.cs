/*******************************************************************************************************
Copyright (c) 2013, Carnegie Mellon University
All rights reserved.
Authors: Anurag Jakhotia<ajakhoti@andrew.cmu.edu>, Prasanna Velagapudi<pkv@cs.cmu.edu>

Redistribution and use in source and binary forms, with or without modification, are permitted provided 
that the following conditions are met:

 -	Redistributions of source code must retain the above copyright notice, this list of conditions and 
    the following disclaimer.
 -	Redistributions in binary form must reproduce the above copyright notice, this list of conditions 
    and the following disclaimer in the documentation and/or other materials provided with the 
    distribution.
 -	Neither the name of Carnegie Mellon University nor the names of its contributors may be used to 
    endorse or promote products derived from this software without specific prior written permission.
 
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED 
WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A 
PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR 
TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF 
ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************************************/
using Microsoft.Kinect;
using Microsoft.Kinect.Face;
using System;
using System.Diagnostics;
using System.ServiceProcess;
using System.Windows.Media;
using System.Collections.Generic;
using Newtonsoft.Json;
using System.Runtime.InteropServices;

using Microsoft.Speech.AudioFormat;
using Microsoft.Speech.Recognition;

namespace PersonalRobotics.Kinect2Server
{
    /// <summary>
    /// A service that publishes data from the Kinect2 over TCP sockets.
    /// </summary>
    /// See: http://msdn.microsoft.com/en-us/library/system.serviceprocess.servicebase(v=vs.110).aspx
    /// 
    public class Kinect2ServerService : ServiceBase
    {
        KinectSensor kinect;
        MultiSourceFrameReader reader;
        AudioSource audioSource;
        AudioBeamFrameReader audioReader;
        FaceFrameSource[] faceFrameSource;
        FaceFrameReader[] faceFrameReader;
        BodyFrameReader bodyReader;

        AsyncNetworkConnector colorConnector;
        AsyncNetworkConnector depthConnector;
        AsyncNetworkConnector irConnector;
        AsyncNetworkConnector bodyConnector;
        AsyncNetworkConnector audioConnector;
        AsyncNetworkConnector faceConnector;

        byte[] colorArray;
        ushort[] depthArray;
        ushort[] irArray;
        byte[] byteColorArray;
        byte[] byteDepthArray;
        byte[] byteIRArray;
        Body[] bodyArray;
        AudioContainer audioContainer;

        //Speech Recognition variables
        private KinectAudioStream convertStream = null;
        private SpeechRecognitionEngine speechEngine = null;

        static readonly int BYTES_PER_COLOR_PIXEL = (PixelFormats.Bgr32.BitsPerPixel + 7) / 8;
        const int BYTES_PER_DEPTH_PIXEL = 2;
        const int BYTES_PER_IR_PIXEL = 2;

        string outputFilePath = @"C:\Users\sarah_000\Documents\building_trust_2016\building-trust-2016-project\k2_server-service\debug_output_a.txt";
       
        public Kinect2ServerService()
        {
            this.ServiceName = "Kinect2Server";
            this.CanStop = true;
            this.CanPauseAndContinue = false;
            this.AutoLog = true;
        }

        /// <summary>
        /// Property that indicates whether the Kinect Server is connected to a sensor.
        /// </summary>
        public bool IsConnected { get { return (this.kinect != null) && kinect.IsAvailable; } }

        /// <summary>
        /// Event that triggers when the server detects a Kinect connection or disconnecting.
        /// </summary>
        public event EventHandler<IsConnectedChangedEventArgs> IsConnectedChanged;

        protected override void OnStart(string[] args)
        {
            //initialize output file
            using(System.IO.StreamWriter file = System.IO.File.CreateText(outputFilePath))
            {
                file.WriteLine("Debug Output: ");
            }
            // Try to open the first available Kinect sensor.
            this.kinect = KinectSensor.GetDefault();
            if (this.kinect == null)
            {
                EventLog.WriteEntry("No Kinect device was detected.");  
                ExitCode = -1;
                throw new KinectException("No kinect device was detected.");
            }
            else
            {
                this.kinect.Open();
                this.kinect.IsAvailableChanged += this.OnAvailableChanged;
            }

            bodyReader = kinect.BodyFrameSource.OpenReader();
            bodyReader.FrameArrived += onBodyFrameArrived;

            // Register as a handler for the image data being returned by the Kinect.
            this.reader = this.kinect.OpenMultiSourceFrameReader(FrameSourceTypes.Color | FrameSourceTypes.Depth | FrameSourceTypes.Infrared | FrameSourceTypes.Body);

            //Setup audio
            this.audioSource = this.kinect.AudioSource;
            //Speech Recognition code
            IReadOnlyList<AudioBeam> audioBeamList = this.kinect.AudioSource.AudioBeams;
            var audioStream = audioBeamList[0].OpenInputStream();
            this.convertStream = new KinectAudioStream(audioStream);
            RecognizerInfo ri = TryGetKinectRecognizer();
            if (ri != null)
            {
                Console.WriteLine("Recognizer not null");
                this.speechEngine = new SpeechRecognitionEngine(ri.Id);
                this.speechEngine.SpeechRecognized += this.SpeechRecognized;
            }
            else
            {
                Console.WriteLine("Null");
            }


            //Setup face
            const FaceFrameFeatures faceFrameFeatures = FaceFrameFeatures.BoundingBoxInColorSpace 
                | FaceFrameFeatures.BoundingBoxInInfraredSpace 
                | FaceFrameFeatures.FaceEngagement 
                | FaceFrameFeatures.Glasses 
                | FaceFrameFeatures.Happy 
                | FaceFrameFeatures.LeftEyeClosed 
                | FaceFrameFeatures.LookingAway 
                | FaceFrameFeatures.MouthMoved 
                | FaceFrameFeatures.MouthOpen 
                | FaceFrameFeatures.PointsInColorSpace 
                | FaceFrameFeatures.PointsInInfraredSpace 
                | FaceFrameFeatures.RightEyeClosed 
                | FaceFrameFeatures.RotationOrientation;

            //this.faceFrameSource = new FaceFrameSource(this.kinect, 0, faceFrameFeatures);
            this.faceFrameSource = new FaceFrameSource[this.kinect.BodyFrameSource.BodyCount];
            this.faceFrameReader = new FaceFrameReader[this.kinect.BodyFrameSource.BodyCount];

            if (this.reader == null)
            {
                EventLog.WriteEntry("Unable to connect to Kinect data stream.");
                ExitCode = -2;
                throw new KinectException("Unable to connect to Kinect data stream.");
            }
            else
            {
                this.reader.MultiSourceFrameArrived += this.OnFrameArrived;
            }
            if (this.audioSource == null)
            {
                EventLog.WriteEntry("Unable to open audio source on kinect");
                ExitCode = -3;
                throw new KinectException("Unable to connect to kinect audio source");
            }
            else
            {
                this.audioReader = this.audioSource.OpenReader();
                if (this.audioReader == null)
                    Console.WriteLine("Issues with audio reader");
                else
                    this.audioReader.FrameArrived += this.onAudioFrameArrived;
            }

            for (var i = 0; i < faceFrameSource.Length; ++i)
            {
                // Register as a handler for the face source data being returned by the Kinect.
                this.faceFrameSource[i] = new FaceFrameSource(this.kinect, 0, faceFrameFeatures);
                if (this.faceFrameSource[i] == null)
                {
                    EventLog.WriteEntry("Unable to create Kinect face source [" + i + "].");
                    ExitCode = -5;
                    throw new KinectException("Unable to create Kinect face source [" + i + "].");
                }

                // Register as a handler for the face reader data being returned by the Kinect.
                this.faceFrameReader[i] = this.faceFrameSource[i].OpenReader();
                if (this.faceFrameReader[i] == null)
                {
                    EventLog.WriteEntry("Unable to create reader for Kinect face source [" + i + "].");
                    ExitCode = -6;
                    throw new KinectException("Unable to create reader for Kinect face source [" + i + "].");
                }
                else
                {
                    this.faceFrameReader[i].FrameArrived += this.onFaceFrameArrived;
                }
            }

            // Allocate storage for the data from the Kinect.
            this.colorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL)];
            this.depthArray = new ushort[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width];
            this.irArray = new ushort[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width];
            this.byteColorArray = new byte[(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL) + sizeof(double)];
            this.byteDepthArray = new byte[this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL + sizeof(double)];
            this.byteIRArray = new byte[this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL + sizeof(double)];
            this.bodyArray = new Body[this.kinect.BodyFrameSource.BodyCount];
            this.audioContainer = new AudioContainer();
            this.audioContainer.samplingFrequency = 16000;
            this.audioContainer.frameLifeTime = 0.016;
            this.audioContainer.numSamplesPerFrame = (int)(this.audioContainer.samplingFrequency * this.audioContainer.frameLifeTime);
            this.audioContainer.numBytesPerSample = sizeof(float);
            this.audioContainer.audioStream = new float[256];
            
            // Create network connectors that will send out the data when it is received.
            this.colorConnector = new AsyncNetworkConnector(Properties.Settings.Default.RgbImagePort);
            this.depthConnector = new AsyncNetworkConnector(Properties.Settings.Default.DepthImagePort);
            this.irConnector = new AsyncNetworkConnector(Properties.Settings.Default.IrImagePort);
            this.bodyConnector = new AsyncNetworkConnector(Properties.Settings.Default.BodyPort);
            this.audioConnector = new AsyncNetworkConnector(Properties.Settings.Default.AudioPort);
            this.faceConnector = new AsyncNetworkConnector(Properties.Settings.Default.FacePort);

            // Open the server connections.
            this.colorConnector.Listen();
            this.depthConnector.Listen();
            this.irConnector.Listen();
            this.bodyConnector.Listen();
            this.audioConnector.Listen();
            this.faceConnector.Listen();
        }

        protected override void OnStop()
        {
            this.kinect.Close();
            this.colorConnector.Close();
            this.depthConnector.Close();
            this.irConnector.Close();
            this.bodyConnector.Close();
            this.audioConnector.Close();
            this.faceConnector.Close();

            this.reader.Dispose(); // TODO: Is this actually necessary?
            this.audioReader.Dispose();
            this.colorConnector.Dispose();
            this.depthConnector.Dispose();
            this.irConnector.Dispose();
            this.bodyConnector.Dispose();
            this.audioConnector.Dispose();
            this.faceConnector.Dispose();
        }
        private void OnFrameArrived(object sender, MultiSourceFrameArrivedEventArgs e)
        {
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            MultiSourceFrame multiSourceFrame = e.FrameReference.AcquireFrame();
            using (ColorFrame colorFrame = multiSourceFrame.ColorFrameReference.AcquireFrame())
            {
                if (colorFrame != null)
                {
                    colorFrame.CopyConvertedFrameDataToArray(this.colorArray, ColorImageFormat.Bgra);
                    System.Buffer.BlockCopy(this.colorArray, 0, this.byteColorArray,0,(this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL));
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteColorArray, (this.kinect.ColorFrameSource.FrameDescription.Height * this.kinect.ColorFrameSource.FrameDescription.Width * BYTES_PER_COLOR_PIXEL), sizeof(double));
                    //this.colorConnector.Broadcast(this.byteColorArray); 
                }
            }

            using (DepthFrame depthFrame = multiSourceFrame.DepthFrameReference.AcquireFrame())
            {
                if (depthFrame != null)
                {
                    depthFrame.CopyFrameDataToArray(this.depthArray);
                    System.Buffer.BlockCopy(this.depthArray, 0, this.byteDepthArray, 0, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteDepthArray, this.kinect.DepthFrameSource.FrameDescription.Height * this.kinect.DepthFrameSource.FrameDescription.Width * BYTES_PER_DEPTH_PIXEL,sizeof(double));
                    //this.depthConnector.Broadcast(this.byteDepthArray);
                }
            }

            using (InfraredFrame irFrame = multiSourceFrame.InfraredFrameReference.AcquireFrame())
            {
                if (irFrame != null)
                {
                    irFrame.CopyFrameDataToArray(this.irArray);
                    System.Buffer.BlockCopy(this.irArray, 0, this.byteIRArray, 0, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL);
                    System.Buffer.BlockCopy(BitConverter.GetBytes(utcTime), 0, this.byteIRArray, this.kinect.InfraredFrameSource.FrameDescription.Height * this.kinect.InfraredFrameSource.FrameDescription.Width * BYTES_PER_IR_PIXEL, sizeof(double));
                    //this.irConnector.Broadcast(this.byteIRArray);
                }
            }      
        }

        private void onAudioFrameArrived(object sender,AudioBeamFrameArrivedEventArgs e)
        {
            Console.WriteLine("Audio Frame arrived");
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            float sqSum = 0.0f;
            int samCount = 0;
            AudioBeamFrameReference audioFrameRefrence = e.FrameReference;
            try
            {
                AudioBeamFrameList frameList = audioFrameRefrence.AcquireBeamFrames();
                if (frameList != null)
                {
                    using (frameList)
                    {
                        IReadOnlyList<AudioBeamSubFrame> subFrameList = frameList[0].SubFrames;
                        using (System.IO.StreamWriter file = System.IO.File.AppendText(outputFilePath))
                        {
                            foreach (AudioBeamSubFrame subFrame in subFrameList)
                            {
                                this.audioContainer.utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
                                this.audioContainer.beamAngle = subFrame.BeamAngle;
                                this.audioContainer.beamAngleConfidence = subFrame.BeamAngleConfidence;
                                byte[] array = new byte[this.audioSource.SubFrameLengthInBytes];
                                subFrame.CopyFrameDataToArray(array);
                                for (int i = 0; i < array.Length; i += sizeof(float))
                                {
                                    audioContainer.audioStream[(int)(i / sizeof(float))] = BitConverter.ToSingle(array, i);
                                }

                                //DIFFERENT RMS

                                float sum = 0.0f;
                                foreach (AudioBodyCorrelation cor in subFrame.AudioBodyCorrelations)
                                {
                                    int bodyCount = kinect.BodyFrameSource.BodyCount;
                                    for (int j = 0; j < bodyCount; j++)
                                    {
                                        if (bodyArray[j].TrackingId == cor.BodyTrackingId)
                                        {
                                            //subFrame.CopyFrameDataToArray(array);
                                             sum = 0;
                                            for (int i = 0; i < audioContainer.audioStream.Length; i += sizeof(float))
                                            {
                                                sqSum += audioContainer.audioStream[i] * audioContainer.audioStream[i];
                                                samCount++;
                                                float meanSquare = sqSum / 40;
                                                sqSum = 0;
                                                samCount = 0;
                                                sum += meanSquare;
                                            }
                                        }
                                    }
                                }

                                string jsonString = JsonConvert.SerializeObject(this.audioContainer);
                                jsonString = jsonString.Remove(jsonString.Length - 1);
                                jsonString += ",\"Correlations\":" + JsonConvert.SerializeObject(subFrame.AudioBodyCorrelations);
                                jsonString += ",\"Time\":" + utcTime;
                                jsonString += ",\"RelativeTime\":" + subFrame.RelativeTime.TotalMilliseconds;
                                jsonString += ",\"Volume\":" + JsonConvert.SerializeObject(sum) + "}";

                                jsonString += "\n";
                                file.WriteLine(jsonString);
                                byte[] transmittedData = new byte[jsonString.Length * sizeof(char)];
                                System.Buffer.BlockCopy(jsonString.ToCharArray(), 0, transmittedData, 0, transmittedData.Length);
                                Console.WriteLine("Sending audio data: ");
                                this.audioConnector.Broadcast(transmittedData);
                                Console.WriteLine("");
                                subFrame.Dispose();
                            }
                        }
                    }
                    frameList.Dispose();
                }
            }
            catch
            {
            }
        }


        void onBodyFrameArrived(object sender, BodyFrameArrivedEventArgs e)
        {
            Console.WriteLine("body frame arrived");
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            using(BodyFrame frame = e.FrameReference.AcquireFrame())
            {
                if(frame != null)
               {
                   bodyArray = new Body[this.kinect.BodyFrameSource.BodyCount];
                   frame.GetAndRefreshBodyData(this.bodyArray);
                   for(var i = 0; i < bodyArray.Length; i++)
                   {
                       Body body = bodyArray[i];
                       if (!body.IsTracked) continue;

                       faceFrameSource[i].TrackingId = body.TrackingId;
                   }

                   List<Body> bodyList = new List<Body>();
                   for(var i = 0; i < bodyArray.Length; ++i)
                   {
                       Body body = bodyArray[i];
                       if (!body.IsTracked) continue;
                       bodyList.Add(body);
                   }

                   Dictionary<string, object> bodyJson = new Dictionary<string, object>{
                                {"Time", utcTime},
                                {"Bodies", bodyList}
                            };

                   //string json = JsonConvert.SerializeObject(bodyJson,
                                //new JsonSerializerSettings { ContractResolver = new BodyContractResolver() }) + "\n";

                   string json = JsonConvert.SerializeObject(bodyJson) + "\n";
                   byte[] bytes = System.Text.Encoding.ASCII.GetBytes(json);
                   this.bodyConnector.Broadcast(bytes);

                   /*string jsonStringBodies = JsonConvert.SerializeObject(this.bodyArray);

                   string jsonStringBodiesAndTime = "{\"Bodies\":" + jsonStringBodies + ", \"Time\":" + utcTime + "}\n";

                   byte[] bodyByteArray = new byte[jsonStringBodiesAndTime.Length * sizeof(char)];
                   System.Buffer.BlockCopy(jsonStringBodiesAndTime.ToCharArray(), 0, bodyByteArray, 0, jsonStringBodiesAndTime.Length * sizeof(char));

                   Console.WriteLine("Sending body data: ");
                   this.bodyConnector.Broadcast(bodyByteArray);
                   Console.WriteLine("");*/
                }
            }
        }

        private void onFaceFrameArrived(object sender, FaceFrameArrivedEventArgs e)
        {
            Console.WriteLine("face frame arrived");
            double utcTime = (DateTime.UtcNow - new DateTime(1970, 1, 1)).TotalSeconds;
            using (FaceFrame frame = e.FrameReference.AcquireFrame())
            {
                if (frame != null)
                {
                    //Console.WriteLine("Frame not null");
                    FaceFrameResult result = frame.FaceFrameResult;
                    if (result != null)
                    {
                        //if (!frame.IsTrackingIdValid) return;
                        //Console.WriteLine("Result not null");
                        String facepoints = JsonConvert.SerializeObject(result.FacePointsInColorSpace);
                        String faceproperties = JsonConvert.SerializeObject(result.FaceProperties);
                        String facerotation = JsonConvert.SerializeObject(result.FaceRotationQuaternion);


                        string jsonStringFace = "{\"Time\":" + utcTime +
                            ",\"TrackingId\":" + result.TrackingId + ",\"Orientation\":" + facerotation +
                            ",\"Properties\":" + faceproperties + ",\"Points\":" + facepoints + "}\n";

                        byte[] faceByteArray = new byte[jsonStringFace.Length * sizeof(char)];
                        System.Buffer.BlockCopy(jsonStringFace.ToCharArray(), 0, faceByteArray, 0, jsonStringFace.Length * sizeof(char));
                        Console.WriteLine("Sending face data: ");
                        this.faceConnector.Broadcast(faceByteArray);
                        Console.WriteLine("");
                    }
                    //else Console.WriteLine("result is null");
                }
                //else Console.WriteLine("frame is null");
            }
        }
        
        //Speech Recognition Code
        private static RecognizerInfo TryGetKinectRecognizer()
        {
            IEnumerable<RecognizerInfo> recognizers;

            // This is required to catch the case when an expected recognizer is not installed.
            // By default - the x86 Speech Runtime is always expected. 
            try
            {
                recognizers = SpeechRecognitionEngine.InstalledRecognizers();
            }
            catch (COMException)
            {
                return null;
            }

            foreach (RecognizerInfo recognizer in recognizers)
            {
                Console.WriteLine(recognizer.Description);
                foreach (KeyValuePair<string, string> entry in recognizer.AdditionalInfo)
                {
                    Console.WriteLine(entry.Key + ": " + entry.Value);
                }
                string value;
                recognizer.AdditionalInfo.TryGetValue("Kinect", out value);
                Console.WriteLine(value);
                if ("True".Equals(value, StringComparison.OrdinalIgnoreCase) &&
                    "en-US".Equals(recognizer.Culture.Name, StringComparison.OrdinalIgnoreCase))
                {
                    return recognizer;
                }
            }

            return null;
        }

        private void SpeechRecognized(object sender, SpeechRecognizedEventArgs e)
        {
            Console.WriteLine("speech recognized");
            const double ConfidenceThreshold = 0.1;
            if(e.Result.Confidence >= ConfidenceThreshold)
            {
                Console.WriteLine(e.Result.Semantics.Value.ToString());
            }
        }


        protected void OnAvailableChanged(object sender, IsAvailableChangedEventArgs e)
        {
            this.IsConnectedChanged(this, new IsConnectedChangedEventArgs(e.IsAvailable));
        }
    }

    /// <summary>
    /// An exception indicating that a Kinect was not detected.
    /// </summary>
    public class KinectException : Exception
    {
        public KinectException()
        {
        }

        public KinectException(string message)
            : base(message)
        {
        }

        public KinectException(string message, Exception inner)
            : base(message, inner)
        {
        }
    }

    /// <summary>
    /// Event triggered where the server connects or disconnects from a Kinect.
    /// </summary>
    public class IsConnectedChangedEventArgs : EventArgs
    {
        bool isConnected;
        public IsConnectedChangedEventArgs(bool isConnected)
        {
            this.isConnected = isConnected;
        }

        public bool IsConnected { get { return isConnected; } }
    }
}
