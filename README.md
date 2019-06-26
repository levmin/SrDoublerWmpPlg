**Sample Rate Doubler**

The SrDoublerWmpPlg a DSP plug-in for Windows Media Player that improves the quality of its audio rendering by doubling the sample rate of its audio streams. This is a special case of a well-known DSP procedure called sample rate conversion (SRC). In music applications, it also sometimes called upsampling. There is a consensus among critical listeners that when properly implemented, upsampling significantly improves the fidelity of music reproduction.

The distinctive features of this plugin are extremely low distortion (below the noise floor of CD Audio streams) and very reasonable CPU utilization. The plug-in works with all versions of Media Player starting with version 10.

On Windows Vista and Windows 7, it is important to configure the audio playback device to
use the doubled sample rate when running in shared mode. This can be done by going to Control Panel/Sound/Properties/Advanced and selecting the rate in the drop box. For example, for audio CDs the proper rate will be &quot;16 bit, 88200 Hz&quot;, and for most DVDs &quot;24 bit, 96000 Hz&quot;. This step is not required when ASIOWmpPlg is installed; see [https://sourceforge.net/projects/asiowmpplg](https://sourceforge.net/projects/asiowmpplg). Many ASIO-enabled audio devices support automatic sample rate adjustment, and those that don&#39;t need to be configured through their vendor-specific audio mixer utilities.
