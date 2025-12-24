
import React from 'react';
import Root from '@theme-original/Root';
import Chatbot from '@site/src/components/Chatbot/Chatbot.js'; // Corrected import path and component name

export default function RootWrapper(props) {
  return (
    <>
      <Root {...props} />
      <Chatbot /> {/* Using the new Chatbot component */}
    </>
  );
}
