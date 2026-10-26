# Third-party notices

## Application source and artwork

OpenTrack Studio application source, its procedural gate drawings and its original demonstration layout are provided under the [MIT License](LICENSE).

The reference application's source was not available and has not been copied. The user-provided reference screenshot is not included in this repository or its distributable. No VelociDrone models, textures, sounds, executable code or other proprietary game assets are bundled. The product name VelociDrone identifies the simulator with which this independent editor seeks to interoperate; it does not imply sponsorship or affiliation.

## Qt 6.8.3

The Windows application dynamically links the Qt Core, Gui, Widgets, Concurrent, Network, OpenGL and OpenGLWidgets libraries as required by the build and loads the relevant Qt platform/image plugins. The optional AI client uses Qt Network and Windows-native Schannel TLS; no OpenSSL runtime is bundled by the packaging helper. Qt is a separate third-party work, not covered by the application's MIT license. Its applicable open-source license terms, including the GNU Lesser General Public License version 3, continue to apply.

- [Qt licensing information](https://doc.qt.io/qt-6/licensing.html)
- [Qt 6.8.3 source archive](https://download.qt.io/archive/qt/6.8/6.8.3/submodules/)
- [Qt Base 6.8.3 source](https://download.qt.io/archive/qt/6.8/6.8.3/submodules/qtbase-everywhere-src-6.8.3.tar.xz)
- [Qt source repository](https://code.qt.io/cgit/qt/qtbase.git/)
- [GNU LGPL version 3](https://www.gnu.org/licenses/lgpl-3.0.html)
- [GNU GPL version 3](https://www.gnu.org/licenses/gpl-3.0.html)
- [Third-party components within Qt](https://doc.qt.io/qt-6/licenses-used-in-qt.html)

The complete corresponding source for the unmodified Qt 6.8.3 modules distributed with this application's Windows package is available in the [official QtBase 6.8.3 source archive](https://download.qt.io/archive/qt/6.8/6.8.3/submodules/qtbase-everywhere-src-6.8.3.tar.xz), including its build files, license texts and bundled third-party sources. The archive SHA-256 is `56001b905601bb9023d399f3ba780d7fa940f3e4861e496a7c490331f49e0b80`, published in the [official checksum file](https://download.qt.io/archive/qt/6.8/6.8.3/submodules/qtbase-everywhere-src-6.8.3.tar.xz.sha256).

Redistributors must retain the applicable Qt and bundled-component license texts and notices. The packaged Qt libraries must remain replaceable in accordance with the applicable LGPL requirements. Recipients may replace the Qt DLLs with ABI-compatible modified builds, or use and modify this application's published source to rebuild it against a modified Qt using the CMake instructions in README.md. This project adds no restriction on reverse engineering needed to debug modifications to the LGPL-covered libraries. The application does not modify Qt itself.

Qt may bundle components such as image codecs and compression libraries under additional licenses. The notices shipped with the exact Qt binaries take precedence over a summary; include those notices when packaging. Runtime DLLs and plugins must come from the same Qt build used to compile the application.

The packaging helper includes the upstream `LICENSES` directory, QtBase attribution metadata and its referenced license/copyright files under `licenses/qt-6.8.3`, together with the SDK's SBOM when present. The notice catalog covers more QtBase components than a particular application build may use. Its included notices cover, among others, FreeType, HarfBuzz, libjpeg-turbo/IJG, libpng, zlib, PCRE2, double-conversion and Unicode data.

This software is based in part on the work of the FreeType Team. This software is based in part on the work of the Independent JPEG Group.

## Windows runtime and BCrypt

Native `.trk` file encryption/decryption uses the Windows BCrypt system API. BCrypt is part of Windows and is not redistributed by this project.

Optional API credential storage uses Windows Credential Manager. Windows Credential Manager and Schannel are system services, not redistributed libraries. DeepSeek is an external API service used only by the opt-in photograph-to-draft workflow; no DeepSeek model weights or client SDK are bundled. API service use remains subject to the provider's own terms.

The Microsoft Visual C++ runtime is separately licensed. The portable packaging helper copies unmodified x64 CRT files only from the licensed build machine's Visual Studio `Redist/MSVC/.../x64/Microsoft.VC143.CRT` directory. It does not source runtime files from the compiler `bin` directory or Windows system folders. Redistribution remains subject to the applicable Microsoft Software License Terms; the application MIT license does not cover these files. See [Microsoft's redistribution guidance](https://learn.microsoft.com/cpp/windows/redistributing-visual-cpp-files), [local deployment guidance](https://learn.microsoft.com/cpp/windows/choosing-a-deployment-method) and [runtime license terms](https://aka.ms/VCRedistLicense).

## File-format interoperability references

The `.trk` adapter is an original C++ implementation using standard Windows cryptographic APIs. Public format observations were checked against primary source implementations, including [FPVTracksideCore's VDFileManager](https://github.com/uewepuep/FPVTracksideCore/blob/master/ExternalData/VDFileManager.cs), whose project provides an [MIT license](https://github.com/uewepuep/FPVTracksideCore/blob/master/LICENSE.md). Reference repositories are not bundled dependencies, and no source or fixtures from repositories lacking an explicit license have been copied into this project.

See [format notes](docs/format.md) for the evidence and the remaining compatibility limits. VelociDrone itself is neither distributed nor licensed by this project.

## Development tools

The GitHub Actions workflow invokes third-party tools under their own terms, including [install-qt-action](https://github.com/jurplel/install-qt-action), [CMake](https://cmake.org/) and the Microsoft compiler. These tools are not application runtime dependencies. Python may be used by the Qt installer; the desktop application's implementation and local image recognition are C++.
