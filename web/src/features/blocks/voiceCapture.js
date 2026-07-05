const getSpeechRecognitionCtor = () =>
  window.SpeechRecognition || window.webkitSpeechRecognition;

export const isSpeechRecognitionSupported = () => !!getSpeechRecognitionCtor();

export const createSpeechRecognizer = ({
  onInterimResult,
  onFinalResult,
  onError,
  onEnd,
} = {}) => {
  const SpeechRecognitionCtor = getSpeechRecognitionCtor();
  if (!SpeechRecognitionCtor) {
    return null;
  }

  const recognition = new SpeechRecognitionCtor();
  recognition.continuous = false;
  recognition.interimResults = true;
  recognition.lang = "en-US";

  recognition.onresult = (event) => {
    let finalTranscript = "";
    let interimTranscript = "";

    for (let i = event.resultIndex; i < event.results.length; i += 1) {
      const result = event.results[i];
      if (result.isFinal) {
        finalTranscript += result[0].transcript;
      } else {
        interimTranscript += result[0].transcript;
      }
    }

    if (finalTranscript) {
      onFinalResult?.(finalTranscript.trim());
    } else if (interimTranscript) {
      onInterimResult?.(interimTranscript.trim());
    }
  };

  recognition.onerror = (event) => {
    onError?.(event.error || "Speech recognition error");
  };

  recognition.onend = () => {
    onEnd?.();
  };

  return {
    start: () => recognition.start(),
    stop: () => recognition.stop(),
  };
};

const WAKE_WORD_PATTERN = /\bmonsieur\b[,:.]?\s*/i;

// Only run the recognized transcript through plan generation once the wake
// word is heard, so ambient background speech doesn't trigger robot actions.
export const extractWakeWordCommand = (transcript) => {
  if (!transcript) return null;
  const match = WAKE_WORD_PATTERN.exec(transcript);
  if (!match) return null;
  return transcript.slice(match.index + match[0].length).trim();
};
