using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Q42.HueApi;
using Q42.HueApi.Interfaces;
using Q42.HueApi.Models;
using Q42.HueApi.Models.Groups;

namespace BodyBasicsWPF
{
    class HueLightsClient
    {
        public HueClient client = new HueClient("192.168.0.40");

        // boa constructor
        public HueLightsClient()
        {
            client.RegisterAsync("test user", "newdeveloper");
            client.Initialize("newdeveloper");
        }
    }
}
