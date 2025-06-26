import { initializeApp } from "firebase/app";
import { getFirestore } from "firebase/firestore";

export const isProduction = process.env.NODE_ENV === "production";
// TODO: Add SDKs for Firebase products that you want to use
// https://firebase.google.com/docs/web/setup#available-libraries
// Your web app's Firebase configuration
const firebaseConfig = {
  apiKey: "AIzaSyDjq80fK7bQx1KeDlsq7LgK2jA6XfCqAG0",
  authDomain: "dronebezorgd-7c265.firebaseapp.com",
  projectId: "dronebezorgd-7c265",
  storageBucket: "dronebezorgd-7c265.firebasestorage.app",
  messagingSenderId: "998287965305",
  appId: "1:998287965305:web:02c766ac781a8ca40268a8",
  measurementId: "G-9Z1YBJVKZ1",
};
// Initialize Firebase

export const IsInDevMode = !isProduction;

const app = initializeApp(firebaseConfig);
const db = getFirestore(app);

export { db };
