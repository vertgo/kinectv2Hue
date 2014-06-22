using System;
using System.Collections.Generic;
using Q42.HueApi;
using System.Diagnostics;
using System.Threading.Tasks;
using System.Linq;


namespace BodyBasicsWPF
{
    public class HueLights
    {
        public int brightness { get; set; }
        public int hue { get; set; }
        public int saturation { get; set; }
        public string whichLight { get; set; }
        HueLightsClient myHueClient = new HueLightsClient();
        IEnumerable<Light> lights;

        public HueLights()
        {
            brightness = 255;
            hue = 127;
            saturation = 127;

            GetLights();
        }


        public async void GetLights()
        {
            try
            {
                lights = await myHueClient.client.GetLightsAsync();
                Debug.WriteLine("get the lights ?" + lights);

                foreach (Light l in lights) {

                    Debug.Write("countin' lights: " + l.Id);
                    
                }
            }
            catch (Exception e)
            {
                Debug.WriteLine("? error::: " + e);
            }
        }

        // on
        public void TurnOn(int light)
        {
            Light thisLight = lights.ElementAt(light);
            Debug.WriteLine("light number on: " + light);

            if (thisLight.State.On == false)
            {
                var command = new LightCommand();
                command.On = true;
                thisLight.State.On = true;
                command.TurnOn();
                myHueClient.client.SendCommandAsync(command, new List<string> { light.ToString() });
            }

        }

        // off
        public void TurnOff(int light)
        {
            Debug.WriteLine("light number off: " + light);
            Light thisLight = lights.ElementAt(light);

            if (thisLight.State.On == true)
            {
                //Debug.WriteLine("turning OFF light!");
                var command = new LightCommand();
                command.On = false;
                thisLight.State.On = false;
                command.TurnOff();
                myHueClient.client.SendCommandAsync(command, new List<string> { light.ToString() });
            }
        }

        // change brightness in a direction
        public static void ChangeBrightness(int light, int direction)
        {
            // 
        }

        // adjust color of a specific light
        public void ChangeHue(int light, string color)
        {
            //Light thisLight = lights.ElementAt(3);

            //if (thisLight.State.On == true)
            //{
            //Debug.WriteLine("turning OFF light!");
            var command = new LightCommand();
            command.On = true;
            command.SetColor(color);
            //thisLight.State.On = true;
            myHueClient.client.SendCommandAsync(command, new List<string> { light.ToString() });
            //}

        }

        // color loop em'
        public static void ColorLoop()
        {

        }

    }
}
