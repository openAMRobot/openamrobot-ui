
### Project Overview
---------

This React-based application provides functionality to control and management of autonomous mobile robots (AMRs)- its primary focus is on providing a user-friendly experience for effective AMR management.

### Prerequisites

*   **Node.js:** Version 18 or higher (use ```sudo apt install nodejs``` to install)
    
*   **npm:** (or yarn) Package manager (use ```sudo apt install npm``` to install)
    
*   **React Developer Tools:** Recommended for development and debugging (follow this [tutorial](https://react.dev/learn/react-developer-tools) to install)
    

### Installation
If you want to start modifying the UI, use ```npm install``` to be assured that every depandancy was downloaded

### Development

#### Running the application locally

To start the development server:

    npm run dev  

This will launch the application in development mode (note that this wont give the full UI as it misses the ROS2 topics). You can access it at http://localhost:3000 (or a different port if specified). The browser will automatically reload when changes are made.

#### Building for production

To create an optimized build for production:

    npm run build  

This will generate a build folder containing the production-ready version of your application. Use ```./compile_ui.bash``` in the main folder to copy it's components in **openamr_ui_package** for accurate testing

### Project Structure

*   **build:** builded files, do not modify

*   **src:** Contains the source code of the application.
    
    *   **app:** Settings for entire project, configs, providers etc. Top app layer.
    
    *   **assets:** assets such as images, fonts, etc
    
    *   **components:** most lower-level logic items of the app
        
    *   **layouts:** website layout layer that wrap pages for reusing components such as header, footer etc.
        
    *   **pages:** pages layer, each of them have their own route, link several components into a coherent logical unit
        
    *   **shared:** entities that can be used on any level of the app: styles, jsons, constants etc
        
    *   **stores:** entities and logic that lives during all website lifecycle and the bus that transfers data between components and pages of the application
        
*   **public:** public resources like libraries, etc. Files that placed to build directory without any conversion or minification
  
    *   **ros:** Contains the libraries to communicate with the ROS2 nodes
    
*   **package.json:** Project dependencies and scripts
    
